/*
 * top_zhengfangxing.h — 正方形循迹状态机
 *
 * 四态：NORMAL(跟线) → ENTRY_WAIT(等待编码器增量) → TURN_L/TURN_R(转弯) → NORMAL
 *
 * 进入转弯：左/右侧三个传感器中有 ≥2 个检测到黑线
 * 退出转弯：中心两个传感器同时检测到黑线
 *
 * ENTRY_WAIT：检测到直角后继续前进，直到编码器增量达到阈值再转弯。
 *   使能和阈值通过 ModeParams entryEnable / entryThreshold 配置。
 */

#ifndef __TOP_ZHENGFANGXING_H
#define __TOP_ZHENGFANGXING_H

#include <stdint.h>

typedef enum {
    TRACK_STATE_NORMAL,
    TRACK_STATE_ENTRY_WAIT,   /* 直角入口等待 */
    TRACK_STATE_TURN_L,
    TRACK_STATE_TURN_R,
    TRACK_STATE_FINISH,       /* 已完成目标直角数，停车 */
} TrackState;

void TrackSquare_Init(void);
void TrackSquare_Task(uint8_t sensorByte);

/* 外部可读当前状态（调试/显示） */
TrackState TrackSquare_GetState(void);
const char *TrackSquare_StateName(void);

/* 已完成的直角计数 */
int16_t TrackSquare_GetTurnCount(void);
void    TrackSquare_ResetTurnCount(void);

#endif /* __TOP_ZHENGFANGXING_H */
