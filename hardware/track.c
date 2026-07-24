/*
 * track.c — 循迹 PID 实现
 *
 * PID 算法参考 TC264 code/common_flie.c 的 pid_compute：
 *   - P: kp * error
 *   - I: 梯形积分 + 抗饱和限幅
 *   - D: 基于测量值微分（避免 setpoint 跳变引入微分冲击）
 */

#include "track.h"

/* ---- PID 状态 ---- */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float prev_error;
    float out_min;
    float out_max;
    float integrator_min;
    float integrator_max;
} TrackPid;

static TrackPid s_pid;

static float   s_lastOffset     = 0.0f;   /* 上次有效偏差，无信号时保持 */

/* 循迹基速：上电默认 12，菜单可修改 */
#define TRACK_BASE_SPEED_DEFAULT  (12)
static int16_t s_trackBaseSpeed  = TRACK_BASE_SPEED_DEFAULT;

/* ---- 默认 PID 参数（可后续通过调试接口修改） ---- */
#define TRACK_KP        (1.0f)
#define TRACK_KI        (0.0f)
#define TRACK_KD        (0.0f)
#define TRACK_OUT_MIN   (-15.0f)
#define TRACK_OUT_MAX   (15.0f)
#define TRACK_IMIN      (-30.0f)
#define TRACK_IMAX      (30.0f)

void Track_Init(void)
{
    s_pid.kp = TRACK_KP;
    s_pid.ki = TRACK_KI;
    s_pid.kd = TRACK_KD;
    s_pid.integrator     = 0.0f;
    s_pid.prev_error     = 0.0f;
    s_pid.out_min        = TRACK_OUT_MIN;
    s_pid.out_max        = TRACK_OUT_MAX;
    s_pid.integrator_min = TRACK_IMIN;
    s_pid.integrator_max = TRACK_IMAX;

    s_lastOffset = 0.0f;
}

/*
 * 质心法计算线位置偏移。
 * sensorByte: bit7=最左侧 ... bit0=最右侧，1=检测到黑线。
 * return: 偏移量（-3.5 ~ +3.5），负=线偏右，正=线偏左。
 *         无有效信号时返回上次值。
 */
float Track_CalcOffset(uint8_t sensorByte)
{
    uint8_t i;
    float   sumPos    = 0.0f;
    float   sumWeight = 0.0f;

    for (i = 0U; i < 8U; i++) {
        if (sensorByte & ((uint8_t)(1U << i))) {
            sumPos    += (float)i;
            sumWeight += 1.0f;
        }
    }

    if (sumWeight < 0.5f) {
        /* 无有效信号，保持上次偏移（让车按最后方向继续尝试找回线） */
        return s_lastOffset;
    }

    s_lastOffset = (sumPos / sumWeight) - 3.5f;
    return s_lastOffset;
}

/*
 * PID 计算（参考 TC264 common_flie.c pid_compute）。
 * setpoint = 0（目标偏差为 0，即对准线中心）。
 * return: 差速修正值（脉冲/20ms），正=左轮加速右轮减速。
 */
float Track_PidUpdate(float measurement, float dt_s)
{
    float error = 0.0f - measurement;   /* setpoint = 0 */

    /* P */
    float p = s_pid.kp * error;

    /* I — 梯形积分 + 抗饱和 */
    s_pid.integrator += 0.5f * s_pid.ki * dt_s * (error + s_pid.prev_error);
    if (s_pid.integrator > s_pid.integrator_max) {
        s_pid.integrator = s_pid.integrator_max;
    }
    if (s_pid.integrator < s_pid.integrator_min) {
        s_pid.integrator = s_pid.integrator_min;
    }

    /* D — 基于测量值微分 */
    float d = 0.0f;
    if (dt_s > 0.0f) {
        float deriv = (measurement - s_pid.prev_error) / dt_s;
        d = -s_pid.kd * deriv;
    }

    float out = p + s_pid.integrator + d;

    /* 输出限幅 */
    if (out > s_pid.out_max) out = s_pid.out_max;
    if (out < s_pid.out_min) out = s_pid.out_min;

    s_pid.prev_error = error;

    return out;
}

/* ---- 在线调参接口 ---- */

float Track_GetKp(void)          { return s_pid.kp; }
void  Track_SetKp(float v)       { if (v >= 0.0f) s_pid.kp = v; }
float Track_GetKi(void)          { return s_pid.ki; }
void  Track_SetKi(float v)       { if (v >= 0.0f) s_pid.ki = v; }
float Track_GetKd(void)          { return s_pid.kd; }
void  Track_SetKd(float v)       { if (v >= 0.0f) s_pid.kd = v; }
float Track_GetOutMax(void)      { return s_pid.out_max; }
void  Track_SetOutMax(float v)
{
    if (v > 0.0f) {
        s_pid.out_max = v;
        s_pid.out_min = -v;
    }
}
float Track_GetIMax(void)        { return s_pid.integrator_max; }
void  Track_SetIMax(float v)
{
    if (v > 0.0f) {
        s_pid.integrator_max = v;
        s_pid.integrator_min = -v;
    }
}

int16_t Track_GetBaseSpeed(void)           { return s_trackBaseSpeed; }
void    Track_SetBaseSpeed(int16_t v)
{
    if (v > 0 && v <= 200) s_trackBaseSpeed = v;
}
