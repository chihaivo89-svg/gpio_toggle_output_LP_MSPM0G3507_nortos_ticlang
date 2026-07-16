#include "speed_control.h"

#include "clock.h"
#include "motor.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 首次调试只允许 30% 输出，每次运行 5 秒后自动停机。 */
#define SPEED_DEFAULT_OUTPUT_LIMIT    (300)
#define SPEED_MAX_OUTPUT_LIMIT        (800)
#define SPEED_MAX_ABS_TARGET          (200)
#define SPEED_MAX_GAIN                (100.0f)
#define SPEED_RUN_TIMEOUT_MS          (5000UL)
#define SPEED_TEST_TIMEOUT_MS         (2000UL)
#define SPEED_MAX_TEST_DUTY           (300)

typedef struct {
    volatile float kp;
    volatile float ki;
    volatile float kd;
    float integral;
    int32_t previousError;
} SpeedPid;

static SpeedPid s_leftPid;
static SpeedPid s_rightPid;

static volatile int32_t s_leftTarget;
static volatile int32_t s_rightTarget;
static volatile int32_t s_leftActual;
static volatile int32_t s_rightActual;
static volatile int32_t s_leftOutput;
static volatile int32_t s_rightOutput;
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
    int32_t outputLimit)
{
    int32_t error;
    float integralCandidate;
    float output;
    float limit = (float)outputLimit;

    if (target == 0) {
        SpeedPid_Reset(pid);
        return 0;
    }

    error = target - actual;
    integralCandidate = pid->integral + pid->ki * (float)error;
    integralCandidate = SpeedPid_Clamp(integralCandidate, -limit, limit);

    output = pid->kp * (float)error + integralCandidate +
             pid->kd * (float)(error - pid->previousError);

    /* 输出已经饱和且误差仍在推动饱和时，本周期不继续累加积分。 */
    if ((output > limit && error > 0) ||
        (output < -limit && error < 0)) {
        integralCandidate = pid->integral;
        output = pid->kp * (float)error + integralCandidate +
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
    s_leftOutput = 0;
    s_rightOutput = 0;

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
    /* No-load baseline tuning at 8 encoder pulses per 20 ms. */
    s_leftPid.kp = 30.0f;
    s_leftPid.ki = 1.5f;
    s_leftPid.kd = 0.0f;
    s_rightPid.kp = 30.0f;
    s_rightPid.ki = 1.5f;
    s_rightPid.kd = 0.0f;

    s_leftTarget = 0;
    s_rightTarget = 0;
    s_leftActual = 0;
    s_rightActual = 0;
    s_outputLimit = SPEED_DEFAULT_OUTPUT_LIMIT;
    s_mode = SPEED_CONTROL_LEFT;
    s_running = false;
    s_openLoopTest = false;
    s_runStartMs = 0;
    s_runDurationMs = SPEED_RUN_TIMEOUT_MS;
    s_testDuty = 0;

    SpeedControl_StopOutputs(true);
}

void SpeedControl_Update20ms(int32_t leftActual, int32_t rightActual)
{
    s_leftActual = leftActual;
    s_rightActual = rightActual;

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
        s_leftOutput = (s_mode == SPEED_CONTROL_LEFT) ? s_testDuty : 0;
        s_rightOutput = (s_mode == SPEED_CONTROL_RIGHT) ? s_testDuty : 0;

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

    /* M3 提供左侧反馈，M4 一比一跟随左侧 PID 输出。 */
    if (s_mode == SPEED_CONTROL_LEFT || s_mode == SPEED_CONTROL_BOTH) {
        s_leftOutput = SpeedPid_Update(
            &s_leftPid, s_leftTarget, leftActual, s_outputLimit);
        Motor_SetSpeed(MOTOR3, s_leftOutput);
        Motor_SetSpeed(MOTOR4, s_leftOutput);
    } else {
        s_leftOutput = 0;
        SpeedPid_Reset(&s_leftPid);
        Motor_Stop(MOTOR3);
        Motor_Stop(MOTOR4);
    }

    /* M1 提供右侧反馈，M2 一比一跟随右侧 PID 输出。 */
    if (s_mode == SPEED_CONTROL_RIGHT || s_mode == SPEED_CONTROL_BOTH) {
        s_rightOutput = SpeedPid_Update(
            &s_rightPid, s_rightTarget, rightActual, s_outputLimit);
        Motor_SetSpeed(MOTOR1, s_rightOutput);
        Motor_SetSpeed(MOTOR2, s_rightOutput);
    } else {
        s_rightOutput = 0;
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
    status->leftActual = s_leftActual;
    status->leftOutput = s_leftOutput;
    status->rightTarget = s_openLoopTest ? 0 : s_rightTarget;
    status->rightActual = s_rightActual;
    status->rightOutput = s_rightOutput;
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
            "msg:OK running side=%c timeout=5s\r\n",
            SpeedControl_ModeCharacter());
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

    if (strcmp(command, "status") == 0) {
        (void)snprintf(reply, replySize,
            "msg:run=%u type=%s side=%c lt=%ld rt=%ld "
            "lkp=%.3f lki=%.3f lkd=%.3f "
            "rkp=%.3f rki=%.3f rkd=%.3f limit=%ld\r\n",
            s_running ? 1U : 0U,
            s_openLoopTest ? "test" : "pid",
            SpeedControl_ModeCharacter(),
            (long)s_leftTarget,
            (long)s_rightTarget,
            (double)s_leftPid.kp,
            (double)s_leftPid.ki,
            (double)s_leftPid.kd,
            (double)s_rightPid.kp,
            (double)s_rightPid.ki,
            (double)s_rightPid.kd,
            (long)s_outputLimit);
        return true;
    }

    if (strcmp(command, "help") == 0) {
        (void)snprintf(reply, replySize,
            "msg:side=l/r/b target=N kp=N ki=N kd=N "
            "limit=N test=N run stop status dump\r\n");
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
