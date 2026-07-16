/*
 *  encoder.h  —— 编码器脉冲计数
 *
 *  测量原理：
 *    B 相上升沿触发捕获中断 → pulseCount++
 *    A 相 GPIO 电平 → 方向判断（前进+1 / 后退-1）
 *    encoder_task() 由 1ms 定时器调用，内部每 20ms 保存一次脉冲值并清零
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/* ---- 单路编码器配置 ---- */
typedef struct {
    GPIO_Regs          *aPort;     /* A 相 GPIO 端口（方向判断） */
    uint32_t            aPin;      /* A 相 GPIO 引脚 */

    GPTIMER_Regs       *captureInst;  /* B 相捕获定时器实例 */
    uint32_t            captureCh;    /* CC 索引 */

    volatile int32_t    pulseCount;   /* 20ms 内累计脉冲数（带符号） */
    volatile int32_t    lastPulses;   /* 上一次 20ms 的脉冲数 */
} Encoder_Cfg;

/* ---- 编码器实例 ---- */
extern Encoder_Cfg gEncMotor1;
extern Encoder_Cfg gEncMotor3;

/* ---- API ---- */

/* 启动捕获定时器 + 使能 NVIC 中断 */
void Encoder_Start(void);

/*
 * 由 TIMER_0 的 5ms 调度槽调用。
 * 每调用 4 次（20ms）保存脉冲值并返回 true，其余时间返回 false。
 */
bool encoder_task(void);

/* 获取最近一次 20ms 的脉冲数 */
static inline int32_t Encoder_GetPulses(const Encoder_Cfg *enc)
{
    return enc->lastPulses;
}

#endif /* __ENCODER_H */
