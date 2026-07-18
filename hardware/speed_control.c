#include "speed_control.h"

#include "clock.h"
#include "motor.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * 普通速度闭环默认允许 80% 输出，每次运行 5 秒后自动停机；
 * 固定 PWM 开环测试仍单独限制为 30%。
 * 默认值直接引用最大闭环限制，避免两处配置再次不一致。
 */
#define SPEED_MAX_OUTPUT_LIMIT        (800)
#define SPEED_DEFAULT_OUTPUT_LIMIT    SPEED_MAX_OUTPUT_LIMIT
#define SPEED_MAX_ABS_TARGET          (200)
#define SPEED_MAX_GAIN                (100.0f)
#define SPEED_RUN_TIMEOUT_MS          (5000UL)
#define SPEED_TEST_TIMEOUT_MS         (2000UL)
#define SPEED_MAX_TEST_DUTY           (300)
#define SPEED_TRIM_SCALE_BASE          (1000)
#define SPEED_DEFAULT_FOLLOWER_TRIM    (75)
#define SPEED_MAX_FOLLOWER_TRIM        (100)
#define SPEED_V2_TARGET_STEP           (1)
#define SPEED_V2_FF_LINEAR_TARGET      (8)
#define SPEED_V2_FF_MAX_TARGET         (20)
#define SPEED_V2_FF_OFFSET             (198.762f)
#define SPEED_V2_FF_GAIN               (11.340f)
#define SPEED_V2_FF_MAX_PERMILLE       (750)
#define SPEED_V2_ANTI_WINDUP_GAIN      (0.25f)
#define SPEED_V2_INTEGRAL_UNLOAD_SCALE (3.0f)
#define SPEED_V2_UNLOAD_MIN_ERROR      (2)
#define SPEED_FILTER_SAMPLE_COUNT      (4U)

typedef struct {
    volatile float kp;
    volatile float ki;
    volatile float kd;
    float integral;
    int32_t previousError;
} SpeedPid;

typedef struct {
    int32_t samples[SPEED_FILTER_SAMPLE_COUNT];
    int32_t sum;
    uint8_t index;
    uint8_t count;
} SpeedActualFilter;

static SpeedPid s_leftPid;
static SpeedPid s_rightPid;
static SpeedActualFilter s_leftActualFilter;
static SpeedActualFilter s_rightActualFilter;

static volatile int32_t s_leftTarget;
static volatile int32_t s_rightTarget;
static volatile int32_t s_leftControlTarget;
static volatile int32_t s_rightControlTarget;
static volatile int32_t s_leftActual;
static volatile int32_t s_rightActual;
static volatile int32_t s_leftFilteredActual;
static volatile int32_t s_rightFilteredActual;
static volatile int32_t s_leftFeedforward;
static volatile int32_t s_rightFeedforward;
static volatile int32_t s_leftOutput;
static volatile int32_t s_rightOutput;
static volatile int32_t s_leftFollowerOutput;
static volatile int32_t s_rightFollowerOutput;
static volatile int32_t s_leftFollowerTrimPermille;
static volatile int32_t s_rightFollowerTrimPermille;
static volatile int32_t s_outputLimit;
static volatile bool s_running;
static volatile bool s_openLoopTest;
static volatile SpeedControl_Mode s_mode;
static volatile uint32_t s_runStartMs;
static volatile uint32_t s_runDurationMs;
static volatile int32_t s_testDuty;

static float SpeedPid_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static int32_t SpeedControl_RoundDivide(int32_t numerator, int32_t denominator)
{
    if (denominator <= 0) {
        return 0;
    }
    if (numerator >= 0) {
        return (numerator + (denominator / 2)) / denominator;
    }
    return (numerator - (denominator / 2)) / denominator;
}

static void SpeedActualFilter_Reset(SpeedActualFilter *filter)
{
    uint8_t index;

    filter->sum = 0;
    filter->index = 0U;
    filter->count = 0U;
    for (index = 0U; index < SPEED_FILTER_SAMPLE_COUNT; index++) {
        filter->samples[index] = 0;
    }
}

/* 四点移动平均仅用于 VOFA 观察，不参与 PID 反馈。 */
static int32_t SpeedActualFilter_Update(
    SpeedActualFilter *filter,
    int32_t actual)
{
    filter->sum -= filter->samples[filter->index];
    filter->samples[filter->index] = actual;
    filter->sum += actual;

    filter->index++;
    if (filter->index >= SPEED_FILTER_SAMPLE_COUNT) {
        filter->index = 0U;
    }
    if (filter->count < SPEED_FILTER_SAMPLE_COUNT) {
        filter->count++;
    }

    return SpeedControl_RoundDivide(filter->sum, filter->count);
}

static int32_t SpeedControl_MoveToward(
    int32_t current,
    int32_t target,
    int32_t step)
{
    if (current < target) {
        int32_t next = current + step;
        return (next > target) ? target : next;
    }
    if (current > target) {
        int32_t next = current - step;
        return (next < target) ? target : next;
    }
    return current;
}

/*
 * 前馈由正向 Target 8~20 的地面稳态数据拟合得到。低于 Target 8
 * 时从零线性过渡，高于 Target 20 不再外推；倒车暂时保持 PI 控制。
 * 前馈还会限制在总输出上限的 75%，为 PID 修正保留余量。
 */
static int32_t SpeedControl_CalculateFeedforward(
    int32_t target,
    int32_t outputLimit)
{
    float absoluteTarget;
    float magnitude;
    float minimumLinearOutput;
    float maximumFeedforward;
    float feedforward;

    if (target <= 0 || outputLimit <= 0) {
        return 0;
    }

    absoluteTarget = (target > SPEED_V2_FF_MAX_TARGET) ?
        (float)SPEED_V2_FF_MAX_TARGET : (float)target;
    minimumLinearOutput = SPEED_V2_FF_OFFSET +
                          SPEED_V2_FF_GAIN *
                          (float)SPEED_V2_FF_LINEAR_TARGET;
    if (absoluteTarget < (float)SPEED_V2_FF_LINEAR_TARGET) {
        magnitude = minimumLinearOutput * absoluteTarget /
                    (float)SPEED_V2_FF_LINEAR_TARGET;
    } else {
        magnitude = SPEED_V2_FF_OFFSET +
                    SPEED_V2_FF_GAIN * absoluteTarget;
    }

    maximumFeedforward = (float)outputLimit *
                         (float)SPEED_V2_FF_MAX_PERMILLE /
                         (float)SPEED_TRIM_SCALE_BASE;
    magnitude = SpeedPid_Clamp(magnitude, 0.0f, maximumFeedforward);
    feedforward = magnitude;

    return (int32_t)(feedforward +
        ((feedforward >= 0.0f) ? 0.5f : -0.5f));
}

/*
 * M4/M2 没有独立编码器，只在双侧前进闭环时允许做静态补偿。
 * trim 单位为千分比：+25 表示 PWM 乘以 1.025，-25 表示乘以 0.975。
 */
static int32_t SpeedControl_ApplyFollowerTrim(
    int32_t output,
    int32_t trimPermille,
    int32_t outputLimit)
{
    int32_t scaled;
    int32_t numerator = output * (SPEED_TRIM_SCALE_BASE + trimPermille);

    if (numerator >= 0) {
        scaled = (numerator + (SPEED_TRIM_SCALE_BASE / 2)) /
                 SPEED_TRIM_SCALE_BASE;
    } else {
        scaled = (numerator - (SPEED_TRIM_SCALE_BASE / 2)) /
                 SPEED_TRIM_SCALE_BASE;
    }

    if (scaled > outputLimit) {
        return outputLimit;
    }
    if (scaled < -outputLimit) {
        return -outputLimit;
    }
    return scaled;
}

static void SpeedPid_Reset(SpeedPid *pid)
{
    pid->integral = 0.0f;
    pid->previousError = 0;
}

/*
 * 离散位置式 PID。ki 已包含 20ms 采样周期，因此调参时直接使用每周期增益。
 * 积分项和最终输出都进行限幅，防止长时间堵转造成积分累积。
 */
static int32_t SpeedPid_Update(
    SpeedPid *pid,
    int32_t target,
    int32_t actual,
    int32_t feedforward,
    int32_t outputLimit)
{
    int32_t error;
    float integralStep;
    float integralCandidate;
    float output;
    float limitedOutput;
    float feedforwardOutput = (float)feedforward;
    float limit = (float)outputLimit;

    if (target == 0) {
        SpeedPid_Reset(pid);
        return 0;
    }

    error = target - actual;
    integralStep = pid->ki * (float)error;

    /*
     * V2 在误差已经反向且幅度足够大时加快卸载已有积分，
     * 减少重负载突然释放后的速度超调；小幅稳态抖动仍按原 Ki 处理。
     */
    if ((pid->integral > 0.0f && error <= -SPEED_V2_UNLOAD_MIN_ERROR) ||
        (pid->integral < 0.0f && error >= SPEED_V2_UNLOAD_MIN_ERROR)) {
        integralStep *= SPEED_V2_INTEGRAL_UNLOAD_SCALE;
    }

    integralCandidate = pid->integral + integralStep;
    integralCandidate = SpeedPid_Clamp(integralCandidate, -limit, limit);

    output = feedforwardOutput +
             pid->kp * (float)error + integralCandidate +
             pid->kd * (float)(error - pid->previousError);

    if (output > limit || output < -limit) {
        /*
         * 用回算法把积分拉回当前执行器可实现的输出，
         * 避免 PWM 饱和期间留下过大的积分。
         */
        limitedOutput = SpeedPid_Clamp(output, -limit, limit);
        integralCandidate += SPEED_V2_ANTI_WINDUP_GAIN *
                             (limitedOutput - output);
        integralCandidate = SpeedPid_Clamp(
            integralCandidate, -limit, limit);
        output = feedforwardOutput +
                 pid->kp * (float)error + integralCandidate +
                 pid->kd * (float)(error - pid->previousError);
    }

    output = SpeedPid_Clamp(output, -limit, limit);

    /* 初次速度调试不允许控制器突然输出与目标相反的方向。 */
    if ((target > 0 && output < 0.0f) ||
        (target < 0 && output > 0.0f)) {
        output = 0.0f;
    }

    pid->integral = integralCandidate;
    pid->previousError = error;

    return (int32_t)(output + ((output >= 0.0f) ? 0.5f : -0.5f));
}

static void SpeedControl_StopOutputs(bool resetPid)
{
    s_openLoopTest = false;
    s_leftControlTarget = 0;
    s_rightControlTarget = 0;
    s_leftFeedforward = 0;
    s_rightFeedforward = 0;
    s_leftOutput = 0;
    s_rightOutput = 0;
    s_leftFollowerOutput = 0;
    s_rightFollowerOutput = 0;

    Motor_Stop(MOTOR1);
    Motor_Stop(MOTOR2);
    Motor_Stop(MOTOR3);
    Motor_Stop(MOTOR4);

    if (resetPid) {
        SpeedPid_Reset(&s_leftPid);
        SpeedPid_Reset(&s_rightPid);
    }
}

static bool SpeedControl_ParseFloat(
    const char *command,
    const char *name,
    float *value)
{
    size_t nameLength = strlen(name);
    char *end;
    float parsed;

    if (strncmp(command, name, nameLength) != 0 ||
        command[nameLength] != '=') {
        return false;
    }

    parsed = strtof(command + nameLength + 1U, &end);
    if (end == command + nameLength + 1U) {
        return false;
    }
    while (isspace((unsigned char)*end)) {
        end++;
    }
    if (*end != '\0') {
        return false;
    }

    *value = parsed;
    return true;
}

static bool SpeedControl_ParseLong(
    const char *command,
    const char *name,
    long *value)
{
    size_t nameLength = strlen(name);
    char *end;
    long parsed;

    if (strncmp(command, name, nameLength) != 0 ||
        command[nameLength] != '=') {
        return false;
    }

    parsed = strtol(command + nameLength + 1U, &end, 10);
    if (end == command + nameLength + 1U) {
        return false;
    }
    while (isspace((unsigned char)*end)) {
        end++;
    }
    if (*end != '\0') {
        return false;
    }

    *value = parsed;
    return true;
}

static bool SpeedControl_GainInRange(float value)
{
    /* 这种写法也会拒绝 NaN。 */
    return value >= 0.0f && value <= SPEED_MAX_GAIN;
}

static bool SpeedControl_TargetInRange(long value)
{
    return value >= -SPEED_MAX_ABS_TARGET &&
           value <= SPEED_MAX_ABS_TARGET;
}

static char SpeedControl_ModeCharacter(void)
{
    if (s_mode == SPEED_CONTROL_RIGHT) {
        return 'R';
    }
    if (s_mode == SPEED_CONTROL_BOTH) {
        return 'B';
    }
    return 'L';
}

void SpeedControl_Init(void)
{
    /* 已通过 Target 8~20 架空及地面测试的正式 PI 参数。 */
    s_leftPid.kp = 30.0f;
    s_leftPid.ki = 1.5f;
    s_leftPid.kd = 0.0f;
    s_rightPid.kp = 30.0f;
    s_rightPid.ki = 1.5f;
    s_rightPid.kd = 0.0f;

    s_leftTarget = 0;
    s_rightTarget = 0;
    s_leftControlTarget = 0;
    s_rightControlTarget = 0;
    s_leftActual = 0;
    s_rightActual = 0;
    s_leftFilteredActual = 0;
    s_rightFilteredActual = 0;
    s_leftFeedforward = 0;
    s_rightFeedforward = 0;
    s_leftFollowerOutput = 0;
    s_rightFollowerOutput = 0;
    /*
     * 道路标定后的正式默认值：降低左前 M4 7.5%，提高右前 M2 7.5%。
     * 串口 trim=N 命令仍可在本次上电期间临时覆盖该默认值。
     */
    s_leftFollowerTrimPermille = -SPEED_DEFAULT_FOLLOWER_TRIM;
    s_rightFollowerTrimPermille = SPEED_DEFAULT_FOLLOWER_TRIM;
    s_outputLimit = SPEED_DEFAULT_OUTPUT_LIMIT;
    s_mode = SPEED_CONTROL_LEFT;
    s_running = false;
    s_openLoopTest = false;
    s_runStartMs = 0;
    s_runDurationMs = SPEED_RUN_TIMEOUT_MS;
    s_testDuty = 0;

    SpeedActualFilter_Reset(&s_leftActualFilter);
    SpeedActualFilter_Reset(&s_rightActualFilter);
    SpeedControl_StopOutputs(true);
}

void SpeedControl_Update20ms(int32_t leftActual, int32_t rightActual)
{
    s_leftActual = leftActual;
    s_rightActual = rightActual;
    s_leftFilteredActual = SpeedActualFilter_Update(
        &s_leftActualFilter, leftActual);
    s_rightFilteredActual = SpeedActualFilter_Update(
        &s_rightActualFilter, rightActual);

    if (!s_running) {
        SpeedControl_StopOutputs(true);
        return;
    }

    if ((uint32_t)(tick_ms - s_runStartMs) >= s_runDurationMs) {
        s_running = false;
        SpeedControl_StopOutputs(true);
        return;
    }

    /* 固定 PWM 测试按左右侧驱动，用于确认同侧两台电机的方向。 */
    if (s_openLoopTest) {
        s_leftControlTarget = 0;
        s_rightControlTarget = 0;
        s_leftFeedforward = 0;
        s_rightFeedforward = 0;
        s_leftOutput = (s_mode == SPEED_CONTROL_LEFT) ? s_testDuty : 0;
        s_rightOutput = (s_mode == SPEED_CONTROL_RIGHT) ? s_testDuty : 0;
        s_leftFollowerOutput = s_leftOutput;
        s_rightFollowerOutput = s_rightOutput;

        if (s_mode == SPEED_CONTROL_LEFT) {
            Motor_SetSpeed(MOTOR3, s_leftOutput);
            Motor_SetSpeed(MOTOR4, s_leftOutput);
            Motor_Stop(MOTOR1);
            Motor_Stop(MOTOR2);
        } else {
            Motor_SetSpeed(MOTOR1, s_rightOutput);
            Motor_SetSpeed(MOTOR2, s_rightOutput);
            Motor_Stop(MOTOR3);
            Motor_Stop(MOTOR4);
        }
        return;
    }

    if (s_mode == SPEED_CONTROL_LEFT || s_mode == SPEED_CONTROL_BOTH) {
        s_leftControlTarget = SpeedControl_MoveToward(
            s_leftControlTarget, s_leftTarget, SPEED_V2_TARGET_STEP);
        s_leftFeedforward = SpeedControl_CalculateFeedforward(
            s_leftControlTarget, s_outputLimit);
    } else {
        s_leftControlTarget = 0;
        s_leftFeedforward = 0;
    }

    if (s_mode == SPEED_CONTROL_RIGHT || s_mode == SPEED_CONTROL_BOTH) {
        s_rightControlTarget = SpeedControl_MoveToward(
            s_rightControlTarget, s_rightTarget, SPEED_V2_TARGET_STEP);
        s_rightFeedforward = SpeedControl_CalculateFeedforward(
            s_rightControlTarget, s_outputLimit);
    } else {
        s_rightControlTarget = 0;
        s_rightFeedforward = 0;
    }

    /* M3 提供左侧反馈，M4 一比一跟随左侧 PID 输出。 */
    if (s_mode == SPEED_CONTROL_LEFT || s_mode == SPEED_CONTROL_BOTH) {
        s_leftOutput = SpeedPid_Update(
            &s_leftPid,
            s_leftControlTarget,
            leftActual,
            s_leftFeedforward,
            s_outputLimit);
        s_leftFollowerOutput = s_leftOutput;
        if (s_mode == SPEED_CONTROL_BOTH &&
            s_leftControlTarget > 0 && s_rightControlTarget > 0) {
            s_leftFollowerOutput = SpeedControl_ApplyFollowerTrim(
                s_leftOutput, s_leftFollowerTrimPermille, s_outputLimit);
        }
        Motor_SetSpeed(MOTOR3, s_leftOutput);
        Motor_SetSpeed(MOTOR4, s_leftFollowerOutput);
    } else {
        s_leftOutput = 0;
        s_leftFollowerOutput = 0;
        SpeedPid_Reset(&s_leftPid);
        Motor_Stop(MOTOR3);
        Motor_Stop(MOTOR4);
    }

    /* M1 提供右侧反馈，M2 一比一跟随右侧 PID 输出。 */
    if (s_mode == SPEED_CONTROL_RIGHT || s_mode == SPEED_CONTROL_BOTH) {
        s_rightOutput = SpeedPid_Update(
            &s_rightPid,
            s_rightControlTarget,
            rightActual,
            s_rightFeedforward,
            s_outputLimit);
        s_rightFollowerOutput = s_rightOutput;
        if (s_mode == SPEED_CONTROL_BOTH &&
            s_leftControlTarget > 0 && s_rightControlTarget > 0) {
            s_rightFollowerOutput = SpeedControl_ApplyFollowerTrim(
                s_rightOutput, s_rightFollowerTrimPermille, s_outputLimit);
        }
        Motor_SetSpeed(MOTOR1, s_rightOutput);
        Motor_SetSpeed(MOTOR2, s_rightFollowerOutput);
    } else {
        s_rightOutput = 0;
        s_rightFollowerOutput = 0;
        SpeedPid_Reset(&s_rightPid);
        Motor_Stop(MOTOR1);
        Motor_Stop(MOTOR2);
    }
}

void SpeedControl_GetStatus(SpeedControl_Status *status)
{
    if (status == NULL) {
        return;
    }

    status->leftTarget = s_openLoopTest ? 0 : s_leftTarget;
    status->leftControlTarget =
        s_openLoopTest ? 0 : s_leftControlTarget;
    status->leftActual = s_leftActual;
    status->leftFilteredActual = s_leftFilteredActual;
    status->leftFeedforward = s_leftFeedforward;
    status->leftOutput = s_leftOutput;
    status->leftFollowerOutput = s_leftFollowerOutput;
    status->leftFollowerTrimPermille = s_leftFollowerTrimPermille;
    status->rightTarget = s_openLoopTest ? 0 : s_rightTarget;
    status->rightControlTarget =
        s_openLoopTest ? 0 : s_rightControlTarget;
    status->rightActual = s_rightActual;
    status->rightFilteredActual = s_rightFilteredActual;
    status->rightFeedforward = s_rightFeedforward;
    status->rightOutput = s_rightOutput;
    status->rightFollowerOutput = s_rightFollowerOutput;
    status->rightFollowerTrimPermille = s_rightFollowerTrimPermille;
    status->running = s_running;
    status->mode = s_mode;
}

bool SpeedControl_ProcessCommand(
    const char *command,
    char *reply,
    uint32_t replySize)
{
    float floatValue;
    long longValue;

    if (command == NULL || reply == NULL || replySize == 0U) {
        return false;
    }

    while (isspace((unsigned char)*command)) {
        command++;
    }

    if (strcmp(command, "stop") == 0 || strcmp(command, "run=0") == 0) {
        s_running = false;
        SpeedControl_StopOutputs(true);
        (void)snprintf(reply, replySize, "msg:OK stopped\r\n");
        return true;
    }

    if (strcmp(command, "run") == 0 || strcmp(command, "run=1") == 0) {
        bool targetReady =
            (s_mode == SPEED_CONTROL_LEFT && s_leftTarget != 0) ||
            (s_mode == SPEED_CONTROL_RIGHT && s_rightTarget != 0) ||
            (s_mode == SPEED_CONTROL_BOTH &&
             s_leftTarget != 0 && s_rightTarget != 0);

        if (!targetReady) {
            (void)snprintf(reply, replySize,
                "msg:ERR set nonzero target first\r\n");
            return false;
        }

        SpeedPid_Reset(&s_leftPid);
        SpeedPid_Reset(&s_rightPid);
        s_openLoopTest = false;
        s_runStartMs = tick_ms;
        s_runDurationMs = SPEED_RUN_TIMEOUT_MS;
        s_running = true;
        (void)snprintf(reply, replySize,
            "msg:OK running side=%c algo=%s timeout=5s\r\n",
            SpeedControl_ModeCharacter(),
            "v2");
        return true;
    }

    if (strcmp(command, "side=l") == 0 ||
        strcmp(command, "side=r") == 0 ||
        strcmp(command, "side=b") == 0) {
        s_running = false;
        SpeedControl_StopOutputs(true);
        s_mode = (command[5] == 'r') ? SPEED_CONTROL_RIGHT :
                 ((command[5] == 'b') ? SPEED_CONTROL_BOTH :
                                         SPEED_CONTROL_LEFT);
        (void)snprintf(reply, replySize,
            "msg:OK side=%c stopped\r\n",
            SpeedControl_ModeCharacter());
        return true;
    }

    if (strcmp(command, "algo=v2") == 0) {
        if (s_running) {
            (void)snprintf(reply, replySize,
                "msg:ERR stop before confirming algo\r\n");
            return false;
        }

        SpeedControl_StopOutputs(true);
        (void)snprintf(reply, replySize,
            "msg:OK algo=v2 official ramp=on ff=on stopped\r\n");
        return true;
    }

    if (strcmp(command, "algostatus") == 0) {
        (void)snprintf(reply, replySize,
            "msg:algo=v2 official ramp_step=%d/20ms ff=%s "
            "antiwindup=backcalc unload=3x filter=%u-sample-display\r\n",
            SPEED_V2_TARGET_STEP,
            "forward-fit-8..20",
            (unsigned int)SPEED_FILTER_SAMPLE_COUNT);
        return true;
    }

    if (strcmp(command, "status") == 0) {
        (void)snprintf(reply, replySize,
            "msg:run=%u type=%s algo=%s side=%c "
            "lt=%ld lct=%ld rt=%ld rct=%ld lff=%ld rff=%ld "
            "lkp=%.3f lki=%.3f lkd=%.3f "
            "rkp=%.3f rki=%.3f rkd=%.3f limit=%ld "
            "m4trim=%ld m2trim=%ld m4out=%ld m2out=%ld\r\n",
            s_running ? 1U : 0U,
            s_openLoopTest ? "test" : "pid",
            "v2",
            SpeedControl_ModeCharacter(),
            (long)s_leftTarget,
            (long)s_leftControlTarget,
            (long)s_rightTarget,
            (long)s_rightControlTarget,
            (long)s_leftFeedforward,
            (long)s_rightFeedforward,
            (double)s_leftPid.kp,
            (double)s_leftPid.ki,
            (double)s_leftPid.kd,
            (double)s_rightPid.kp,
            (double)s_rightPid.ki,
            (double)s_rightPid.kd,
            (long)s_outputLimit,
            (long)s_leftFollowerTrimPermille,
            (long)s_rightFollowerTrimPermille,
            (long)s_leftFollowerOutput,
            (long)s_rightFollowerOutput);
        return true;
    }

    if (strcmp(command, "trimstatus") == 0) {
        (void)snprintf(reply, replySize,
            "msg:trim forward-both-only m4trim=%ld scale=%ld/1000 out=%ld "
            "m2trim=%ld scale=%ld/1000 out=%ld\r\n",
            (long)s_leftFollowerTrimPermille,
            (long)(SPEED_TRIM_SCALE_BASE + s_leftFollowerTrimPermille),
            (long)s_leftFollowerOutput,
            (long)s_rightFollowerTrimPermille,
            (long)(SPEED_TRIM_SCALE_BASE + s_rightFollowerTrimPermille),
            (long)s_rightFollowerOutput);
        return true;
    }

    if (strcmp(command, "help") == 0) {
        (void)snprintf(reply, replySize,
            "msg:side=l/r/b target=N kp=N ki=N kd=N "
            "limit=N trim=N m2trim=N m4trim=N trimstatus "
            "algo=v2 algostatus test=N run stop status dump\r\n");
        return true;
    }

    if (SpeedControl_ParseLong(command, "trim", &longValue) ||
        SpeedControl_ParseLong(command, "m2trim", &longValue) ||
        SpeedControl_ParseLong(command, "m4trim", &longValue)) {
        if (s_running) {
            (void)snprintf(reply, replySize,
                "msg:ERR stop before changing trim\r\n");
            return false;
        }
        if (longValue < -SPEED_MAX_FOLLOWER_TRIM ||
            longValue > SPEED_MAX_FOLLOWER_TRIM) {
            (void)snprintf(reply, replySize,
                "msg:ERR trim range -%d..%d per-mille\r\n",
                SPEED_MAX_FOLLOWER_TRIM, SPEED_MAX_FOLLOWER_TRIM);
            return false;
        }

        if (strncmp(command, "m2trim=", 7U) == 0) {
            s_rightFollowerTrimPermille = (int32_t)longValue;
        } else if (strncmp(command, "m4trim=", 7U) == 0) {
            s_leftFollowerTrimPermille = (int32_t)longValue;
        } else {
            /* 正值用于修正向右偏：左前 M4 减速，右前 M2 加速。 */
            s_leftFollowerTrimPermille = -(int32_t)longValue;
            s_rightFollowerTrimPermille = (int32_t)longValue;
        }

        (void)snprintf(reply, replySize,
            "msg:OK m4trim=%ld m2trim=%ld forward-both-only\r\n",
            (long)s_leftFollowerTrimPermille,
            (long)s_rightFollowerTrimPermille);
        return true;
    }

    if (SpeedControl_ParseLong(command, "test", &longValue)) {
        if (s_mode == SPEED_CONTROL_BOTH) {
            (void)snprintf(reply, replySize,
                "msg:ERR select side=l or side=r for test\r\n");
            return false;
        }
        if (longValue < -SPEED_MAX_TEST_DUTY ||
            longValue > SPEED_MAX_TEST_DUTY || longValue == 0) {
            (void)snprintf(reply, replySize,
                "msg:ERR test range -%d..-1 or 1..%d\r\n",
                SPEED_MAX_TEST_DUTY, SPEED_MAX_TEST_DUTY);
            return false;
        }

        s_running = false;
        SpeedControl_StopOutputs(true);
        s_testDuty = (int32_t)longValue;
        s_openLoopTest = true;
        s_runStartMs = tick_ms;
        s_runDurationMs = SPEED_TEST_TIMEOUT_MS;
        s_running = true;
        (void)snprintf(reply, replySize,
            "msg:OK test side=%c duty=%ld timeout=2s\r\n",
            SpeedControl_ModeCharacter(), longValue);
        return true;
    }

    if (SpeedControl_ParseLong(command, "target", &longValue) ||
        SpeedControl_ParseLong(command, "lt", &longValue) ||
        SpeedControl_ParseLong(command, "rt", &longValue)) {
        if (!SpeedControl_TargetInRange(longValue)) {
            (void)snprintf(reply, replySize,
                "msg:ERR target range -%d..%d\r\n",
                SPEED_MAX_ABS_TARGET, SPEED_MAX_ABS_TARGET);
            return false;
        }

        if (strncmp(command, "lt=", 3U) == 0) {
            s_leftTarget = (int32_t)longValue;
        } else if (strncmp(command, "rt=", 3U) == 0) {
            s_rightTarget = (int32_t)longValue;
        } else {
            s_leftTarget = (int32_t)longValue;
            s_rightTarget = (int32_t)longValue;
        }
        (void)snprintf(reply, replySize,
            "msg:OK lt=%ld rt=%ld\r\n",
            (long)s_leftTarget, (long)s_rightTarget);
        return true;
    }

    if (SpeedControl_ParseLong(command, "limit", &longValue)) {
        if (longValue < 0 || longValue > SPEED_MAX_OUTPUT_LIMIT) {
            (void)snprintf(reply, replySize,
                "msg:ERR limit range 0..%d\r\n",
                SPEED_MAX_OUTPUT_LIMIT);
            return false;
        }
        s_outputLimit = (int32_t)longValue;
        (void)snprintf(reply, replySize,
            "msg:OK limit=%ld\r\n", longValue);
        return true;
    }

    if (SpeedControl_ParseFloat(command, "kp", &floatValue) ||
        SpeedControl_ParseFloat(command, "ki", &floatValue) ||
        SpeedControl_ParseFloat(command, "kd", &floatValue) ||
        SpeedControl_ParseFloat(command, "lkp", &floatValue) ||
        SpeedControl_ParseFloat(command, "lki", &floatValue) ||
        SpeedControl_ParseFloat(command, "lkd", &floatValue) ||
        SpeedControl_ParseFloat(command, "rkp", &floatValue) ||
        SpeedControl_ParseFloat(command, "rki", &floatValue) ||
        SpeedControl_ParseFloat(command, "rkd", &floatValue)) {
        if (!SpeedControl_GainInRange(floatValue)) {
            (void)snprintf(reply, replySize,
                "msg:ERR gain range 0..100\r\n");
            return false;
        }

        if (strncmp(command, "kp=", 3U) == 0) {
            s_leftPid.kp = floatValue;
            s_rightPid.kp = floatValue;
        } else if (strncmp(command, "ki=", 3U) == 0) {
            s_leftPid.ki = floatValue;
            s_rightPid.ki = floatValue;
        } else if (strncmp(command, "kd=", 3U) == 0) {
            s_leftPid.kd = floatValue;
            s_rightPid.kd = floatValue;
        } else if (strncmp(command, "lkp=", 4U) == 0) {
            s_leftPid.kp = floatValue;
        } else if (strncmp(command, "lki=", 4U) == 0) {
            s_leftPid.ki = floatValue;
        } else if (strncmp(command, "lkd=", 4U) == 0) {
            s_leftPid.kd = floatValue;
        } else if (strncmp(command, "rkp=", 4U) == 0) {
            s_rightPid.kp = floatValue;
        } else if (strncmp(command, "rki=", 4U) == 0) {
            s_rightPid.ki = floatValue;
        } else {
            s_rightPid.kd = floatValue;
        }

        (void)snprintf(reply, replySize,
            "msg:OK gain=%.3f\r\n", (double)floatValue);
        return true;
    }

    (void)snprintf(reply, replySize,
        "msg:ERR unknown command\r\n");
    return false;
}
