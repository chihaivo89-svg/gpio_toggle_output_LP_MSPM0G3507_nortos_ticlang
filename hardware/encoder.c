/*
 *  encoder.c  —— 编码器脉冲计数实现
 *
 *  motor1 B相捕获: TIMG0 (E1_B), A相: PA13 (E1_A)
 *  motor3 B相捕获: TIMA1 (E3_B), A相: PB5  (E3_A)
 */

#include "encoder.h"
#include "clock.h"   /* tick_ms（如需） */

/* ================================================================
 *  编码器配置表
 * ================================================================ */

Encoder_Cfg gEncMotor1 = {
    .aPort        = ECO_E1_A_PORT,
    .aPin         = ECO_E1_A_PIN,
    .captureInst  = E1_B_INST,
    .captureCh    = DL_TIMER_CC_0_INDEX,
};

Encoder_Cfg gEncMotor3 = {
    .aPort        = ECO_E3_A_PORT,
    .aPin         = ECO_E3_A_PIN,
    .captureInst  = E3_B_INST,
    .captureCh    = DL_TIMER_CC_0_INDEX,
};

/* ---- 1ms 累加器 ---- */
static uint16_t s_encTick = 0;

/* ================================================================
 *  编码器 B 相捕获中断
 * ================================================================ */

/* motor1 B 相 (TIMG0) */
void E1_B_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(E1_B_INST)) {
        case DL_TIMER_IIDX_CC0_DN: {
            /* 读 A 相判断方向 */
            uint32_t aLevel = DL_GPIO_readPins(gEncMotor1.aPort, gEncMotor1.aPin);
            if (aLevel) {
                gEncMotor1.pulseCount++;
            } else {
                gEncMotor1.pulseCount--;
            }
            break;
        }
        default:
            break;
    }
}

/* motor3 B 相 (TIMA1) */
void E3_B_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(E3_B_INST)) {
        case DL_TIMER_IIDX_CC0_DN: {
            uint32_t aLevel = DL_GPIO_readPins(gEncMotor3.aPort, gEncMotor3.aPin);
            if (aLevel) {
                gEncMotor3.pulseCount--;
            } else {
                gEncMotor3.pulseCount++;
            }
            break;
        }
        default:
            break;
    }
}

/* ================================================================
 *  encoder_task —— 1ms 定时器回调，每 20ms 保存一次脉冲数据
 * ================================================================ */

void encoder_task(void)
{
    s_encTick++;
    if (s_encTick < 20) {
        return;     /* 未到 20ms */
    }
    s_encTick = 0;

    /* 保存并清零脉冲计数 */
    gEncMotor1.lastPulses = gEncMotor1.pulseCount;
    gEncMotor1.pulseCount = 0;

    gEncMotor3.lastPulses = gEncMotor3.pulseCount;
    gEncMotor3.pulseCount = 0;
}

/* ================================================================
 *  启动捕获定时器
 * ================================================================ */

void Encoder_Start(void)
{
    /* 启动定时器计数（syscfg 配置为 startTimer=STOP） */
    DL_TimerG_startCounter(E1_B_INST);
    DL_TimerA_startCounter(E3_B_INST);

    /* 使能 NVIC 中断 */
    NVIC_EnableIRQ(E1_B_INST_INT_IRQN);
    NVIC_EnableIRQ(E3_B_INST_INT_IRQN);
}
