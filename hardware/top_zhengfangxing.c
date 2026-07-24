/*
 * top_zhengfangxing.c — 正方形循迹状态机实现
 *
 * 传感器布局（8路，gRxByte）：
 *   bit7   bit6   bit5   bit4   bit3   bit2   bit1   bit0
 *  [左3]  [左2]  [左1]  [中R]  [中L]  [右1]  [右2]  [右3]
 *
 * 直角检测：一侧 4 个传感器全部检测到黑线（左半 0xF0，右半 0x0F）。
 * 转弯速度由 ModeParams trackTurnSpeed 控制（菜单可调）。
 * 入口等待使能/阈值由 entryEnable / entryThreshold 控制。
 */

#include "top_zhengfangxing.h"
#include "track.h"
#include "speed_control.h"
#include "param_storage.h"
#include "menu.h"
#include "encoder.h"

static TrackState s_state = TRACK_STATE_NORMAL;

/* 入口等待：记录检测到直角时的 M1 累计脉冲和转弯方向 */
static int32_t  s_entryBasePulse = 0;
static uint8_t  s_entryTurnLeft  = 1U;   /* 1=左转, 0=右转 */

/* 直角完成计数 */
static int16_t s_turnCount = 0;

void TrackSquare_Init(void)
{
    s_state = TRACK_STATE_NORMAL;
    s_turnCount = 0;
}

void TrackSquare_Task(uint8_t sensorByte)
{
    /* 参数从当前 mode 的参数池读取 */
    const ModeParams *p  = &g_modeParams[g_menuMode - 1U];
    int16_t baseSpd = p->trackBaseSpeed;
    int16_t turnSpd = p->trackTurnSpeed;

    switch (s_state) {

        case TRACK_STATE_NORMAL:
            /* ── 正常循迹 PID ── */
            {
                float offset = Track_CalcOffset(sensorByte);
                float diff   = Track_PidUpdate(offset, 0.01f);
                int16_t left  = baseSpd + (int16_t)diff;
                int16_t right = baseSpd - (int16_t)diff;

                if (left  < 0) left  = 0;
                if (right < 0) right = 0;

                SpeedControl_SetTargets(left, right);
            }

            /* 检测直角入口（仅 entryEnable 开启时才处理） */
            if (p->entryEnable) {
                /* 一侧 4 个传感器全部检测到黑线：左半 0xF0，右半 0x0F */
                if ((sensorByte & 0xF0U) == 0xF0U) {
                    s_entryBasePulse = Encoder_GetTotalPulses(&gEncMotor1);
                    s_entryTurnLeft  = 1U;
                    s_state = TRACK_STATE_ENTRY_WAIT;
                } else if ((sensorByte & 0x0FU) == 0x0FU) {
                    s_entryBasePulse = Encoder_GetTotalPulses(&gEncMotor1);
                    s_entryTurnLeft  = 0U;
                    s_state = TRACK_STATE_ENTRY_WAIT;
                }
            }
            /* entryEnable == 0：完全忽略直角，仅循迹 */
            break;

        case TRACK_STATE_ENTRY_WAIT:
            /* ── 继续循迹，等待编码器增量达到阈值 ── */
            {
                float offset = Track_CalcOffset(sensorByte);
                float diff   = Track_PidUpdate(offset, 0.01f);
                int16_t left  = baseSpd + (int16_t)diff;
                int16_t right = baseSpd - (int16_t)diff;

                if (left  < 0) left  = 0;
                if (right < 0) right = 0;

                SpeedControl_SetTargets(left, right);
            }

            {
                int32_t delta = Encoder_GetTotalPulses(&gEncMotor1) - s_entryBasePulse;
                if (delta < 0) delta = -delta;   /* 正反兼容 */
                if (delta >= p->entryThreshold) {
                    s_state = s_entryTurnLeft ? TRACK_STATE_TURN_L : TRACK_STATE_TURN_R;
                }
            }
            break;

        case TRACK_STATE_TURN_L:
            /* ── 缓慢左转 ── */
            SpeedControl_SetTargets(-turnSpd, turnSpd);

            /* 退出：最左侧传感器（bit7）再次检测到黑线 */
            if (sensorByte & 0x80U) {
                s_turnCount++;
                if (p->stopAfterTurns > 0 && s_turnCount >= p->stopAfterTurns) {
                    s_state = TRACK_STATE_FINISH;
                } else {
                    s_state = TRACK_STATE_NORMAL;
                }
            }
            break;

        case TRACK_STATE_TURN_R:
            /* ── 缓慢右转 ── */
            SpeedControl_SetTargets(turnSpd, -turnSpd);

            /* 退出：最右侧传感器（bit0）再次检测到黑线 */
            if (sensorByte & 0x01U) {
                s_turnCount++;
                if (p->stopAfterTurns > 0 && s_turnCount >= p->stopAfterTurns) {
                    s_state = TRACK_STATE_FINISH;
                } else {
                    s_state = TRACK_STATE_NORMAL;
                }
            }
            break;

        case TRACK_STATE_FINISH:
            /* ── 已完成目标直角数，停车 ── */
            SpeedControl_SetTargets(0, 0);
            break;
    }
}

TrackState TrackSquare_GetState(void)
{
    return s_state;
}

const char *TrackSquare_StateName(void)
{
    switch (s_state) {
        case TRACK_STATE_NORMAL:     return "NORM";
        case TRACK_STATE_ENTRY_WAIT: return "WAIT";
        case TRACK_STATE_TURN_L:     return "TURN_L";
        case TRACK_STATE_TURN_R:     return "TURN_R";
        case TRACK_STATE_FINISH:     return "STOP";
        default:                     return "?";
    }
}

int16_t TrackSquare_GetTurnCount(void)
{
    return s_turnCount;
}

void TrackSquare_ResetTurnCount(void)
{
    s_turnCount = 0;
}
