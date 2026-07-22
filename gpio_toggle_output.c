/*
 * MSPM0G3507 四轮小车主程序
 *
 * 1ms 定时器负责按键扫描，并用 5 个时间槽调度编码器、IMU 和速度环。
 * UART1（PA9=RX、PA8=TX）保留给后续循迹数据，当前收到一个字节后回传。
 * OLED（SSD1306，I2C）：SDA=PA16，SCL=PA15。
 */

#include <stdio.h>

#include "ti_msp_dl_config.h"
#include "clock.h"
#include "encoder.h"
#include "key.h"
#include "oled_hardware_i2c.h"
#include "speed_control.h"
#include "heading_control.h"
#include "vehicle_yaw.h"
#include "IMU660RB/imu660rb.h"
#include "menu.h"
#include "ttop_funtion.h"

/* ==================== 主程序状态 ==================== */

/* UART1 循迹数据预留。 */
volatile uint8_t gRxByte = 0;
volatile bool gDataReceived = false;

static uint8_t sched_tick = 0;
static bool speed_sample_ready = false;

/* ==================== 前台任务 ==================== */

/* 当前只回传循迹串口收到的单字节，后续可在这里接入数据协议。 */
static void App_ProcessTrackUart(void)
{
    if (gDataReceived) {
        gDataReceived = false;
        DL_UART_Main_transmitDataBlocking(UART_0_INST, gRxByte);
    }
}

/* ==================== 中断与周期调度 ==================== */

void SysTick_Handler(void)
{
    tick_ms++;
}

/* TIMER_0 每 1ms 进入一次，五个槽位循环调度。 */
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMERA_IIDX_ZERO:
            Key_Update1ms();

            switch (sched_tick) {
                case 0: /* 编码器任务每 5ms 调用，内部每 20ms 产生一次样本。 */
                    if (encoder_task()) {
                        speed_sample_ready = true;
                    }
                    Read_IMU660RB();
                    VehicleYaw_Update5ms(angular_rate_mdps[1]);
                    break;

                case 1:
                    FusionTasks();
                    break;

                case 2:
                    if (speed_sample_ready) {
                        int32_t leftActual =
                            Encoder_GetPulses(&gEncMotor3);
                        int32_t rightActual =
                            Encoder_GetPulses(&gEncMotor1);

                        speed_sample_ready = false;
                        HeadingControl_Update20ms();
                        SpeedControl_Update20ms(leftActual, rightActual);
                    }
                    break;

                case 3:
                    ModeExecTask();
                    break;

                case 4:
                    break;
            }

            sched_tick++;
            if (sched_tick >= 5U) {
                sched_tick = 0U;
            }
            break;

        default:
            break;
    }
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            gRxByte = DL_UART_Main_receiveData(UART_0_INST);
            gDataReceived = true;
            break;

        default:
            break;
    }
}

/* ==================== 初始化与主循环 ==================== */

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    /*
     * 正式速度环上电后保持停止。
     * 后续菜单或 IMU 外环通过 SetTargets、Start、Stop 接口控制。
     */
    SpeedControl_Init();

    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    Key_Init();
    Encoder_Start();
    OLED_Init();
    IMU660RB_Init();
    VehicleYaw_Init();
    HeadingControl_Init();

    Menu_Init();

    while (1) {
        App_ProcessTrackUart();
        Menu_HandleKeys();
        Menu_Draw();
    }
}
