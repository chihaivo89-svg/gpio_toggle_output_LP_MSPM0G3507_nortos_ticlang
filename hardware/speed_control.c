#include "speed_control.h"

#include "motor.h"
#include "speed_control_config.h"

/*
 * 正式 V2 速度环实现。
 *
 * 参数统一放在 speed_control_config.h；本文件只保留状态、算法和电机映射。
 * 左侧 M3、右侧 M1 使用编码器闭环，M4、M2 跟随同侧闭环输出。
 */

/* ---- 在线调参运行时变量（菜单可修改） ---- */
float   g_speedPidKp       = SPEED_PID_KP;
float   g_speedPidKi       = SPEED_PID_KI;
float   g_speedPidKd       = SPEED_PID_KD;
int16_t g_speedPidOutLimit = SPEED_OUTPUT_LIMIT;

typedef struct {
    float integral;
    float previousError;
} SpeedPid;

/* 只保留跨 20ms 控制周期必须保存的状态。 */
static SpeedPid s_leftPid;
static SpeedPid s_rightPid;

static volatile int32_t s_leftTarget;
static volatile int32_t s_rightTarget;
static volatile float s_leftControlTarget;
static volatile float s_rightControlTarget;
static volatile float s_differentialTargetOffset;
static volatile bool s_running;

/* ==================== 基础工具函数 ==================== */

static float SpeedControl_ClampFloat(
    float value,
    float minimum,
    float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static float SpeedControl_MoveToward(
    float current,
    float target,
    float step)
{
    if (current < target) {
        float next = current + step;
        return (next > target) ? target : next;
    }
    if (current > target) {
        float next = current - step;
        return (next < target) ? target : next;
    }
    return current;
}

static float SpeedControl_ClampTarget(float target)
{
    if (target > (float)SPEED_MAX_ABS_TARGET) {
        return (float)SPEED_MAX_ABS_TARGET;
    }
    if (target < -(float)SPEED_MAX_ABS_TARGET) {
        return -(float)SPEED_MAX_ABS_TARGET;
    }
    return target;
}

/*
 * Base targets belong to the normal speed-control API.  The heading
 * outer loop never changes them; it only creates this per-sample effective
 * target pair while the vehicle is driving forward.
 */
static void SpeedControl_GetEffectiveTargets(
    float *leftTarget,
    float *rightTarget)
{
    if (s_leftTarget > 0 && s_rightTarget > 0) {
        *leftTarget = SpeedControl_ClampTarget(
            (float)s_leftTarget + s_differentialTargetOffset);
        *rightTarget = SpeedControl_ClampTarget(
            (float)s_rightTarget - s_differentialTargetOffset);
    } else {
        *leftTarget = (float)s_leftTarget;
        *rightTarget = (float)s_rightTarget;
    }
}

/* ==================== V2 前馈与 PID ==================== */

/*
 * 正向前馈由 Target 8~20 的地面稳态数据拟合得到。
 * Target 8 以下从零线性过渡，20 以上不再外推；倒车暂不使用前馈。
 * 前馈最多占输出上限的 75%，为 PI 修正保留余量。
 */
static int32_t SpeedControl_CalculateFeedforward(float target)
{
    float absoluteTarget;
    float magnitude;
    float minimumLinearOutput;
    float maximumFeedforward;

    if (target <= 0.0f) {
        return 0;
    }

    absoluteTarget = (target > (float)SPEED_FF_MAX_TARGET) ?
        (float)SPEED_FF_MAX_TARGET : (float)target;
    minimumLinearOutput =
        SPEED_FF_OFFSET + SPEED_FF_GAIN * (float)SPEED_FF_LINEAR_TARGET;

    if (absoluteTarget < (float)SPEED_FF_LINEAR_TARGET) {
        magnitude = minimumLinearOutput * absoluteTarget /
                    (float)SPEED_FF_LINEAR_TARGET;
    } else {
        magnitude = SPEED_FF_OFFSET + SPEED_FF_GAIN * absoluteTarget;
    }

    maximumFeedforward =
        (float)g_speedPidOutLimit * (float)SPEED_FF_MAX_PERMILLE /
        (float)SPEED_TRIM_SCALE_BASE;
    magnitude = SpeedControl_ClampFloat(
        magnitude, 0.0f, maximumFeedforward);

    return (int32_t)(magnitude + 0.5f);
}

static int32_t SpeedControl_ApplyFollowerTrim(
    int32_t output,
    int32_t trimPermille)
{
    int32_t scaled;
    int32_t numerator =
        output * (SPEED_TRIM_SCALE_BASE + trimPermille);

    if (numerator >= 0) {
        scaled = (numerator + (SPEED_TRIM_SCALE_BASE / 2)) /
                 SPEED_TRIM_SCALE_BASE;
    } else {
        scaled = (numerator - (SPEED_TRIM_SCALE_BASE / 2)) /
                 SPEED_TRIM_SCALE_BASE;
    }

    if (scaled > g_speedPidOutLimit) {
        return g_speedPidOutLimit;
    }
    if (scaled < -g_speedPidOutLimit) {
        return -g_speedPidOutLimit;
    }
    return scaled;
}

static void SpeedPid_Reset(SpeedPid *pid)
{
    pid->integral = 0.0f;
    pid->previousError = 0.0f;
}

/*
 * V2 离散位置式 PI/PID。
 *
 * - 前馈承担大部分稳态输出；
 * - 反向误差达到 2 个脉冲时，积分以 3 倍速度卸载；
 * - 输出饱和时用回算法修正积分，避免解除阻力后产生大超调。
 */
static int32_t SpeedPid_Update(
    SpeedPid *pid,
    float target,
    int32_t actual,
    int32_t feedforward)
{
    float error;
    float integralStep;
    float integralCandidate;
    float output;
    float limitedOutput;
    const float limit = (float)g_speedPidOutLimit;

    if (target == 0.0f) {
        SpeedPid_Reset(pid);
        return 0;
    }

    error = target - (float)actual;
    integralStep = g_speedPidKi * error;

    if ((pid->integral > 0.0f && error <= -SPEED_UNLOAD_MIN_ERROR) ||
        (pid->integral < 0.0f && error >= SPEED_UNLOAD_MIN_ERROR)) {
        integralStep *= SPEED_INTEGRAL_UNLOAD_SCALE;
    }

    integralCandidate = pid->integral + integralStep;
    integralCandidate = SpeedControl_ClampFloat(
        integralCandidate, -limit, limit);

    output = (float)feedforward +
             g_speedPidKp * error +
             integralCandidate +
             g_speedPidKd * (error - pid->previousError);

    if (output > limit || output < -limit) {
        limitedOutput = SpeedControl_ClampFloat(output, -limit, limit);
        integralCandidate +=
            SPEED_ANTI_WINDUP_GAIN * (limitedOutput - output);
        integralCandidate = SpeedControl_ClampFloat(
            integralCandidate, -limit, limit);

        output = (float)feedforward +
                 g_speedPidKp * error +
                 integralCandidate +
                 g_speedPidKd * (error - pid->previousError);
    }

    output = SpeedControl_ClampFloat(output, -limit, limit);

    /* 不允许控制输出与当前目标方向相反。 */
    if ((target > 0 && output < 0.0f) ||
        (target < 0 && output > 0.0f)) {
        output = 0.0f;
    }

    pid->integral = integralCandidate;
    pid->previousError = error;

    return (int32_t)(output + ((output >= 0.0f) ? 0.5f : -0.5f));
}

/* ==================== 输出管理 ==================== */

static void SpeedControl_StopOutputs(void)
{
    s_leftControlTarget = 0;
    s_rightControlTarget = 0;
    s_differentialTargetOffset = 0.0f;

    SpeedPid_Reset(&s_leftPid);
    SpeedPid_Reset(&s_rightPid);
    Motor_Stop(MOTOR1);
    Motor_Stop(MOTOR2);
    Motor_Stop(MOTOR3);
    Motor_Stop(MOTOR4);
}

/* ==================== 对外接口 ==================== */

void SpeedControl_Init(void)
{
    s_leftTarget = 0;
    s_rightTarget = 0;
    s_differentialTargetOffset = 0.0f;
    s_running = false;

    SpeedControl_StopOutputs();
}

bool SpeedControl_SetTargets(int32_t leftTarget, int32_t rightTarget)
{
    if (leftTarget < -SPEED_MAX_ABS_TARGET ||
        leftTarget > SPEED_MAX_ABS_TARGET ||
        rightTarget < -SPEED_MAX_ABS_TARGET ||
        rightTarget > SPEED_MAX_ABS_TARGET) {
        return false;
    }

    s_leftTarget = leftTarget;
    s_rightTarget = rightTarget;
    s_differentialTargetOffset = 0.0f;
    return true;
}

void SpeedControl_SetDifferentialTargetOffsetFloat(float offset)
{
    s_differentialTargetOffset = SpeedControl_ClampTarget(offset);
}

bool SpeedControl_Start(void)
{
    if (s_leftTarget == 0 && s_rightTarget == 0) {
        return false;
    }

    s_running = false;
    SpeedControl_StopOutputs();
    s_running = true;
    return true;
}

void SpeedControl_Stop(void)
{
    s_running = false;
    SpeedControl_StopOutputs();
}

bool SpeedControl_IsRunning(void)
{
    return s_running;
}

void SpeedControl_Update20ms(int32_t leftActual, int32_t rightActual)
{
    float leftEffectiveTarget;
    float rightEffectiveTarget;
    int32_t leftFeedforward;
    int32_t rightFeedforward;
    int32_t leftOutput;
    int32_t rightOutput;
    int32_t leftFollowerOutput;
    int32_t rightFollowerOutput;

    if (!s_running) {
        return;
    }

    SpeedControl_GetEffectiveTargets(
        &leftEffectiveTarget, &rightEffectiveTarget);
    s_leftControlTarget = SpeedControl_MoveToward(
        s_leftControlTarget, leftEffectiveTarget, (float)SPEED_TARGET_STEP);
    s_rightControlTarget = SpeedControl_MoveToward(
        s_rightControlTarget, rightEffectiveTarget, (float)SPEED_TARGET_STEP);

    if (leftEffectiveTarget == 0.0f && rightEffectiveTarget == 0.0f &&
        s_leftControlTarget == 0.0f && s_rightControlTarget == 0.0f) {
        SpeedControl_Stop();
        return;
    }

    leftFeedforward =
        SpeedControl_CalculateFeedforward(s_leftControlTarget);
    rightFeedforward =
        SpeedControl_CalculateFeedforward(s_rightControlTarget);

    /* M3、M1 是有编码器反馈的闭环电机。 */
    leftOutput = SpeedPid_Update(
        &s_leftPid, s_leftControlTarget, leftActual, leftFeedforward);
    rightOutput = SpeedPid_Update(
        &s_rightPid, s_rightControlTarget, rightActual, rightFeedforward);

    /*
     * M4、M2 没有独立编码器，跟随同侧闭环输出。
     * 道路标定的静态补偿只在双侧都向前时生效。
     */
    leftFollowerOutput = leftOutput;
    rightFollowerOutput = rightOutput;
    if (s_leftControlTarget > 0.0f && s_rightControlTarget > 0.0f) {
        leftFollowerOutput = SpeedControl_ApplyFollowerTrim(
            leftOutput, SPEED_M4_TRIM_PERMILLE);
        rightFollowerOutput = SpeedControl_ApplyFollowerTrim(
            rightOutput, SPEED_M2_TRIM_PERMILLE);
    }

    Motor_SetSpeed(MOTOR3, leftOutput);
    Motor_SetSpeed(MOTOR4, leftFollowerOutput);
    Motor_SetSpeed(MOTOR1, rightOutput);
    Motor_SetSpeed(MOTOR2, rightFollowerOutput);
}

/* ---- 在线调参接口 ---- */

float SpeedControl_GetKp(void)           { return g_speedPidKp; }
void  SpeedControl_SetKp(float v)        { if (v >= 0.0f) g_speedPidKp = v; }
float SpeedControl_GetKi(void)           { return g_speedPidKi; }
void  SpeedControl_SetKi(float v)        { if (v >= 0.0f) g_speedPidKi = v; }
float SpeedControl_GetKd(void)           { return g_speedPidKd; }
void  SpeedControl_SetKd(float v)        { if (v >= 0.0f) g_speedPidKd = v; }
int16_t SpeedControl_GetOutLimit(void)   { return g_speedPidOutLimit; }
void    SpeedControl_SetOutLimit(int16_t v)
{
    if (v > 0 && v <= 1000) g_speedPidOutLimit = v;
}
