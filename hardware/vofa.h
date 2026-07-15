#ifndef __VOFA_H
#define __VOFA_H

#include <stdint.h>

/*
 * VOFA FireWater 六通道数据：
 * I0 左侧目标，I1 M3 实际脉冲，I2 左侧 PID 输出，
 * I3 右侧目标，I4 M1 实际脉冲，I5 右侧 PID 输出。
 */
typedef struct {
    int32_t leftTarget;
    int32_t leftActual;
    int32_t leftOutput;
    int32_t rightTarget;
    int32_t rightActual;
    int32_t rightOutput;
} VOFA_SpeedData;

/* SysTick 启动后初始化 VOFA 发送节拍。 */
void VOFA_Init(void);

/* 主循环调用；每 40ms 发送一帧速度 PID 观测数据。 */
void VOFA_Update(const VOFA_SpeedData *data);

#endif /* __VOFA_H */
