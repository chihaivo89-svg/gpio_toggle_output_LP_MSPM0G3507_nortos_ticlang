/*
 *  encoder.h  —— 编码器脉冲计数
 *
 *  测量原理：
 *    B 相上升沿触发捕获中断，并读取 A 相 GPIO 判断该边沿方向。
 *    正向和反向边沿分别累计；每 20ms 由多数边沿确定真实方向，
 *    再以正反边沿总数作为速度大小，避免少量 A 相误判抵消有效脉冲。
 *
 *  例如真实正转 20 个边沿，其中 2 个被误判为反向：
 *    旧净计数为 18 - 2 = 16，会让速度环误以为电机突然减速；
 *    当前重建结果为 +(18 + 2) = 20，不引入额外滤波延迟。
 *    真正反转时反向边沿占多数，结果仍保持负数。
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

    volatile int32_t    lastPulses;   /* 重建后的上一次 20ms 有符号脉冲数 */
    volatile uint32_t   positiveEdges;     /* 当前 20ms 内判为正向的边沿 */
    volatile uint32_t   negativeEdges;     /* 当前 20ms 内判为反向的边沿 */
    volatile uint32_t   lastPositiveEdges; /* 上一次 20ms 的正向边沿数 */
    volatile uint32_t   lastNegativeEdges; /* 上一次 20ms 的反向边沿数 */
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

static inline uint32_t Encoder_GetPositiveEdges(const Encoder_Cfg *enc)
{
    return enc->lastPositiveEdges;
}

static inline uint32_t Encoder_GetNegativeEdges(const Encoder_Cfg *enc)
{
    return enc->lastNegativeEdges;
}

#endif /* __ENCODER_H */
