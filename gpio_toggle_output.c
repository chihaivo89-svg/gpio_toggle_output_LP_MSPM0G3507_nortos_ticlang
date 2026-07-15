/*
 *  UART 接收 -> OLED 实时显示
 *
 *  串口：UART1, PA9=RX, PA8=TX；每收到 1 字节，回传并刷新 OLED。
 *  OLED (SSD1306, I2C): SDA=PA16, SCL=PA15
 */

#include <stdio.h>
#include "ti_msp_dl_config.h"
#include "oled_hardware_i2c.h"
#include "clock.h"
#include "motor.h"
#include "encoder.h"
#include "key.h"
#include "vofa.h"
#include "IMU660RB/imu660rb.h"

/* ---- UART ---- */
volatile uint8_t  gRxByte       = 0;
volatile bool     gDataReceived = false;

/* ---- OLED 显示缓冲区 ---- */
static char oled_buf[24];

/* ---- TIMER_0 测试：ms 计数 ---- */
volatile uint32_t gTimerTest    = 0;
//static uint32_t   gLastSpeedSet = 0;

/* ---- 5ms 主调度节拍 ---- */
static uint8_t sched_tick = 0;

/* OLED 每行只能显示 16 个 8x8 字符，显示值限制到 9999ms。 */
static unsigned int Key_GetDisplayTimeMs(Key_Id key)
{
    uint16_t time = Key_GetPressedTimeMs(key);

    return (time > 9999U) ? 9999U : (unsigned int)time;
}

/* ===================== SysTick 中断 ===================== */

void SysTick_Handler(void)
{
    tick_ms++;
}

/* ===================== TIMER_0 中断：1ms ===================== */

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMERA_IIDX_ZERO:
            gTimerTest++;
            /* 每 1ms 扫描并消抖 KEY1~KEY4。 */
            Key_Update1ms();

            switch (sched_tick) {
                case 0:  /* 0ms: 编码器 + IMU 读取 */
                    encoder_task();
                    //DL_GPIO_setPins(TEST_IMU_PORT, TEST_IMU_PIN);
                    Read_IMU660RB();
                    //DL_GPIO_clearPins(TEST_IMU_PORT, TEST_IMU_PIN);
                    break;

                case 1:  /* 1ms: 姿态解算 */
                    //DL_GPIO_setPins(TEST_Fusion_PORT, TEST_Fusion_PIN);
                    FusionTasks();
                    //DL_GPIO_clearPins(TEST_Fusion_PORT, TEST_Fusion_PIN);
                    break;

                case 2:  /* 2ms: 电机速度 PID（预留） */
                    break;

                case 3:  /* 3ms: IMU PID / 循迹 PID（预留） */
                    break;

                case 4:  /* 4ms: 空闲 */
                    break;
            }

            if (++sched_tick >= 5) sched_tick = 0;
            break;
        default:
            break;
    }
}

/* ===================== UART 中断 ===================== */

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            gRxByte       = DL_UART_Main_receiveData(UART_0_INST);
            gDataReceived = true;
            break;
        default:
            break;
    }
}

/* ===================== 主函数 ===================== */

int main(void)
{
    VOFA_SpeedData vofaSpeedData = {0};

    SYSCFG_DL_init();

    SysTick_Init();

    //DL_UART_Main_enableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    Key_Init();
    VOFA_Init();

    /* 启动编码器捕获 */
    Encoder_Start();

    OLED_Init();
    //OLED_Clear();

    /* 初始化 IMU660RB */
    IMU660RB_Init();

    /* 标题 */
    OLED_ShowString(0, 0, (uint8_t *)"track:", 8);
    OLED_ShowString(0, 1, (uint8_t *)"spd1:", 8);
    OLED_ShowString(0, 2, (uint8_t *)"spd2:", 8);

    /* motor1 固定 50% 正转 */
    //Motor_SetSpeed(MOTOR1, 500);
    //Motor_SetSpeed(MOTOR2, 500);
    //Motor_SetSpeed(MOTOR3, 500);
    //Motor_SetSpeed(MOTOR4, 500);

    while (1) {
        /*
         * 当前先观察编码器原始速度：单位为脉冲/20ms。
         * 目标速度和 PID 输出保持 0，不会在此处启动任何电机。
         */
        vofaSpeedData.leftActual  = Encoder_GetPulses(&gEncMotor3);
        vofaSpeedData.rightActual = Encoder_GetPulses(&gEncMotor1);
        VOFA_Update(&vofaSpeedData);

        if (gDataReceived) {
            gDataReceived = false;
            DL_UART_Main_transmitDataBlocking(UART_0_INST, gRxByte);
        }

        OLED_ShowNum(32, 0, gRxByte, 6, 8);
        //OLED_ShowNum(32, 3, gTimerTest, 8, 16);
        OLED_ShowSignedNum(32, 1, Encoder_GetPulses(&gEncMotor1), 5, 8);
        OLED_ShowSignedNum(32, 2, Encoder_GetPulses(&gEncMotor3), 5, 8);

        /* 显示欧拉角 - 第 3~5 行 */
        sprintf((char *)oled_buf, "Pitch:%6.1f", euler.angle.pitch);
        OLED_ShowString(0, 3, (uint8_t *)oled_buf, 8);
        sprintf((char *)oled_buf, " Roll:%6.1f", euler.angle.roll);
        OLED_ShowString(0, 4, (uint8_t *)oled_buf, 8);
        sprintf((char *)oled_buf, "  Yaw:%6.1f", euler.angle.yaw);
        OLED_ShowString(0, 5, (uint8_t *)oled_buf, 8);

        /* 显示 KEY1~KEY4 当前稳定按下的时间，单位 ms；松开后自动清零。 */
        sprintf((char *)oled_buf, "K1:%04u K2:%04u", Key_GetDisplayTimeMs(KEY_ID_1), Key_GetDisplayTimeMs(KEY_ID_2));
        OLED_ShowString(0, 6, (uint8_t *)oled_buf, 8);
        sprintf((char *)oled_buf, "K3:%04u K4:%04u", Key_GetDisplayTimeMs(KEY_ID_3), Key_GetDisplayTimeMs(KEY_ID_4));
        OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);
    }
}
