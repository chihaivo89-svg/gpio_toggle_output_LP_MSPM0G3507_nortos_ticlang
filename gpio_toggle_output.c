/*
 *  UART 接收 -> OLED 实时显示
 *
 *  串口：UART1, PA9=RX, PA8=TX；每收到 1 字节，回传并刷新 OLED。
 *  OLED (SSD1306, I2C): SDA=PA16, SCL=PA15
 */

#include <stdio.h>
#include <string.h>
#include "ti_msp_dl_config.h"
#include "oled_hardware_i2c.h"
#include "clock.h"
#include "motor.h"
#include "encoder.h"
#include "key.h"
#include "road_test.h"
#include "speed_control.h"
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
static bool speed_sample_ready = false;
/* 使用静态快照，避免在 512 字节中断栈上放置完整状态结构。 */
static SpeedControl_Status speed_sample_status;

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
                    if (encoder_task()) {
                        speed_sample_ready = true;
                    }
                    //DL_GPIO_setPins(TEST_IMU_PORT, TEST_IMU_PIN);
                    Read_IMU660RB();
                    //DL_GPIO_clearPins(TEST_IMU_PORT, TEST_IMU_PIN);
                    break;

                case 1:  /* 1ms: 姿态解算 */
                    //DL_GPIO_setPins(TEST_Fusion_PORT, TEST_Fusion_PIN);
                    FusionTasks();
                    //DL_GPIO_clearPins(TEST_Fusion_PORT, TEST_Fusion_PIN);
                    break;

                case 2:  /* 2ms: 有新编码器数据时执行一次 20ms 速度 PID */
                    if (speed_sample_ready) {
                        speed_sample_ready = false;
                        SpeedControl_Update20ms(
                            Encoder_GetPulses(&gEncMotor3),
                            Encoder_GetPulses(&gEncMotor1));
                        SpeedControl_GetStatus(&speed_sample_status);
                        /* 脱机测试只在这里保存一条 20ms 整数样本。 */
                        RoadTest_Record20ms(&speed_sample_status);
                    }
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
    SpeedControl_Status speedStatus;
    char vofaCommand[VOFA_COMMAND_MAX_LENGTH];
    char vofaReply[192];

    SYSCFG_DL_init();

    SysTick_Init();

    /* 闭环默认停机，必须通过 VOFA 的 run 命令才会驱动电机。 */
    SpeedControl_Init();

    //DL_UART_Main_enableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    Key_Init();
    VOFA_Init();

    /* 启动编码器捕获 */
    Encoder_Start();

    OLED_Init();
    //OLED_Clear();

    /* KEY1 脱机直线测试默认空闲，初始化不会启动任何电机。 */
    RoadTest_Init();

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
        /* 先处理 KEY1 脱机测试状态，再读取可能已经变化的闭环状态。 */
        SpeedControl_GetStatus(&speedStatus);
        RoadTest_Update(&speedStatus);
        SpeedControl_GetStatus(&speedStatus);

        /* 保持 VOFA 原有 I0~I5 通道顺序。 */
        vofaSpeedData.leftTarget  = speedStatus.leftTarget;
        vofaSpeedData.leftActual  = speedStatus.leftActual;
        vofaSpeedData.leftOutput  = speedStatus.leftOutput;
        vofaSpeedData.rightTarget = speedStatus.rightTarget;
        vofaSpeedData.rightActual = speedStatus.rightActual;
        vofaSpeedData.rightOutput = speedStatus.rightOutput;
        VOFA_Update(&vofaSpeedData);

        /* 在线参数命令在主循环解析，不在串口中断中执行浮点运算。 */
        if (VOFA_ReadCommand(vofaCommand, sizeof(vofaCommand))) {
            if (strcmp(vofaCommand, "dump") == 0) {
                /* 回放最后一次有效脱机测试的 20ms 速度波形。 */
                RoadTest_DumpToVofa();
            } else if (strcmp(vofaCommand, "dumpall") == 0) {
                /* 快速导出本批次已经完成的全部测试，不按 20ms 延时回放。 */
                RoadTest_DumpAllToVofa();
            } else if (RoadTest_ProcessCommand(
                           vofaCommand, vofaReply, sizeof(vofaReply))) {
                VOFA_SendMessage(vofaReply);
            } else {
                (void)SpeedControl_ProcessCommand(
                    vofaCommand, vofaReply, sizeof(vofaReply));
                VOFA_SendMessage(vofaReply);
            }
        }

        if (gDataReceived) {
            gDataReceived = false;
            DL_UART_Main_transmitDataBlocking(UART_0_INST, gRxByte);
        }

        /* 脱机测试期间 OLED 由 RoadTest 独占，避免两个页面相互覆盖。 */
        if (!RoadTest_UsesOled()) {
            OLED_ShowString(0, 0, (uint8_t *)"track:", 8);
            OLED_ShowString(0, 1, (uint8_t *)"spd1:", 8);
            OLED_ShowString(0, 2, (uint8_t *)"spd2:", 8);
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
}
