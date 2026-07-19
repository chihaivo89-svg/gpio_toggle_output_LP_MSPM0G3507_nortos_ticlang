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

/*
 * 编码器容错重建：
 *
 * 一个 20ms 窗口内电机方向不会反复切换，因此真实方向应由绝大多数
 * 边沿共同决定。M3 的 A 相如果在少数 B 相捕获瞬间被误读，旧的正负
 * 相减会让一个误判同时“少一个正脉冲、又多一个负脉冲”，速度误差为 2。
 *
 * 这里用多数边沿确定符号，用全部边沿确定大小。这样不做时间平均，
 * 不增加速度环相位延迟；真正反转时负向边沿占多数，仍返回负速度。
 */
static int32_t Encoder_ReconstructPulses(
    uint32_t positiveEdges,
    uint32_t negativeEdges)
{
    int32_t totalEdges = (int32_t)(positiveEdges + negativeEdges);

    if (positiveEdges > negativeEdges) {
        return totalEdges;
    }
    if (negativeEdges > positiveEdges) {
        return -totalEdges;
    }
    return 0;
}

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
                gEncMotor1.positiveEdges++;
            } else {
                gEncMotor1.negativeEdges++;
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
                gEncMotor3.negativeEdges++;
            } else {
                gEncMotor3.positiveEdges++;
            }
            break;
        }
        default:
            break;
    }
}

/* ================================================================
 *  encoder_task —— 5ms 调度槽回调，每 20ms 保存一次脉冲数据
 * ================================================================ */

bool encoder_task(void)
{
    uint32_t interruptState;
    uint32_t motor1Positive;
    uint32_t motor1Negative;
    uint32_t motor3Positive;
    uint32_t motor3Negative;

    s_encTick++;
    if (s_encTick < 4) {    /* 5ms × 4 = 20ms */
        return false;     /* 未到 20ms */
    }
    s_encTick = 0;

    /*
     * 原子地取得本窗口计数并清零。临界区只包含几次 32 位读写，
     * 防止捕获中断刚好落在“读取—清零”之间造成边沿丢失。
     */
    interruptState = __get_PRIMASK();
    __disable_irq();
    motor1Positive = gEncMotor1.positiveEdges;
    motor1Negative = gEncMotor1.negativeEdges;
    motor3Positive = gEncMotor3.positiveEdges;
    motor3Negative = gEncMotor3.negativeEdges;
    gEncMotor1.positiveEdges = 0U;
    gEncMotor1.negativeEdges = 0U;
    gEncMotor3.positiveEdges = 0U;
    gEncMotor3.negativeEdges = 0U;
    if (interruptState == 0U) {
        __enable_irq();
    }

    gEncMotor1.lastPositiveEdges = motor1Positive;
    gEncMotor1.lastNegativeEdges = motor1Negative;
    gEncMotor1.lastPulses = Encoder_ReconstructPulses(
        motor1Positive, motor1Negative);

    gEncMotor3.lastPositiveEdges = motor3Positive;
    gEncMotor3.lastNegativeEdges = motor3Negative;
    gEncMotor3.lastPulses = Encoder_ReconstructPulses(
        motor3Positive, motor3Negative);

    return true;
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
