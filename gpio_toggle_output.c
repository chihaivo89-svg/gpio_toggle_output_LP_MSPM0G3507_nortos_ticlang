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
#include "speed_control_config.h"
#include "heading_control.h"
#include "vehicle_yaw.h"
#include "IMU660RB/imu660rb.h"

/* ==================== 主程序状态 ==================== */

/* UART1 循迹数据预留。 */
volatile uint8_t gRxByte = 0;
volatile bool gDataReceived = false;

static char oled_buf[24];
static uint8_t sched_tick = 0;
static bool speed_sample_ready = false;

static bool s_lastKey1Pressed = false;
static bool s_key1StartPending = false;
static unsigned long s_key1StartRequestMs;

/* ==================== 前台任务 ==================== */

/* OLED 每行最多显示 16 个 8x8 字符，按下时间限制到 9999ms。 */
static unsigned int Key_GetDisplayTimeMs(Key_Id key)
{
    uint16_t time = Key_GetPressedTimeMs(key);

    return (time > 9999U) ? 9999U : (unsigned int)time;
}

/* 当前只回传循迹串口收到的单字节，后续可在这里接入数据协议。 */
static void App_ProcessTrackUart(void)
{
    if (gDataReceived) {
        gDataReceived = false;
        DL_UART_Main_transmitDataBlocking(UART_0_INST, gRxByte);
    }
}

/* 正式运行停止入口：先撤销航向差速，再立即停止四个电机。 */
static void App_StopVehicle(void)
{
    s_key1StartPending = false;
    HeadingControl_Stop();
    SpeedControl_Stop();
}

/*
 * 正式运行启动入口：
 *   1. 使用启动前连续静止 300ms 的平均航向作为直行参考；
 *   2. 启动 target=18 的速度环；
 *   3. 启动航向外环，并持续运行到 KEY1 再次按下。
 */
static bool App_StartVehicle(void)
{
    float referenceDeg;

    if (SpeedControl_IsRunning()) {
        return false;
    }

    if (!VehicleYaw_IsCalibrated()) {
        return false;
    }

    /*
     * Capture the reference before PWM starts.  The rolling window is valid
     * only after the car has remained still for the most recent 300 ms.
     */
    if (!VehicleYaw_GetStationaryReferenceDeg(&referenceDeg)) {
        return false;
    }

    if (!SpeedControl_SetTargets(
            SPEED_DEFAULT_TARGET, SPEED_DEFAULT_TARGET)) {
        return false;
    }

    if (!SpeedControl_Start()) {
        return false;
    }

    if (!HeadingControl_Start(referenceDeg)) {
        SpeedControl_Stop();
        return false;
    }

    return true;
}

/*
 * KEY1 一键启停状态机：
 *   停止时按下：只登记启动请求，等待 2 秒后启动，不阻塞主循环。
 *   等待时再按：取消启动请求，电机保持停止。
 *   运行时按下：直接清零四路 PWM 和 PID 状态，立即停止，不做减速斜坡。
 */
static void App_HandleKeyToggle(void)
{
    bool key1Pressed = Key_IsPressed(KEY_ID_1);

    /* 消抖后的按下沿：松开 → 按下只触发一次。 */
    if (key1Pressed && !s_lastKey1Pressed) {
        if (SpeedControl_IsRunning()) {
            App_StopVehicle();
        } else if (s_key1StartPending) {
            s_key1StartPending = false;
        } else {
            s_key1StartRequestMs = tick_ms;
            s_key1StartPending = true;
        }
    }

    /*
     * 无阻塞延时：主循环在等待期间仍持续刷新按键、OLED 和串口。
     * unsigned 减法可正确处理 tick_ms 自然回绕。
     */
    if (s_key1StartPending &&
        (tick_ms - s_key1StartRequestMs) >= SPEED_KEY1_START_DELAY_MS) {
        s_key1StartPending = false;
        (void)App_StartVehicle();
    }

    s_lastKey1Pressed = key1Pressed;
}

/* OLED 只负责状态显示，不参与速度环计算。 */
static void App_UpdateOled(void)
{
    const char *key1State;

    if (s_key1StartPending) {
        key1State = "WAIT";
    } else {
        key1State = SpeedControl_IsRunning() ? "RUN " : "STOP";
    }

    OLED_ShowString(0, 0, (uint8_t *)"track:", 8);
    OLED_ShowString(0, 1, (uint8_t *)"spd1:", 8);
    OLED_ShowString(0, 2, (uint8_t *)"spd2:", 8);
    OLED_ShowNum(32, 0, gRxByte, 6, 8);
    OLED_ShowSignedNum(
        32, 1, Encoder_GetPulses(&gEncMotor1), 5, 8);
    OLED_ShowSignedNum(
        32, 2, Encoder_GetPulses(&gEncMotor3), 5, 8);

    sprintf(oled_buf, "Pitch:%6.1f", euler.angle.pitch);
    OLED_ShowString(0, 3, (uint8_t *)oled_buf, 8);
    sprintf(oled_buf, " Roll:%6.1f", euler.angle.roll);
    OLED_ShowString(0, 4, (uint8_t *)oled_buf, 8);
    sprintf(oled_buf, "  Yaw:%6.1f", euler.angle.yaw);
    OLED_ShowString(0, 5, (uint8_t *)oled_buf, 8);

    sprintf(
        oled_buf,
        "K1:%s K2:%04u",
        key1State,
        Key_GetDisplayTimeMs(KEY_ID_2));
    OLED_ShowString(0, 6, (uint8_t *)oled_buf, 8);
    sprintf(
        oled_buf,
        "K3:%04u K4:%04u",
        Key_GetDisplayTimeMs(KEY_ID_3),
        Key_GetDisplayTimeMs(KEY_ID_4));
    OLED_ShowString(0, 7, (uint8_t *)oled_buf, 8);
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
                    /* 预留给 IMU 航向环或循迹外环。 */
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

    while (1) {
        App_ProcessTrackUart();
        App_HandleKeyToggle();
        App_UpdateOled();
    }
}
