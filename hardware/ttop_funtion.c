/*
 * ttop_funtion.c — 模式执行模块
 *
 * 四个模式函数当前为空壳，后续填充具体逻辑。
 * ModeExecTask() 在 TIMER_0 ISR 的槽 3 中每 5ms 调用一次。
 */

#include "ttop_funtion.h"
#include "menu.h"
#include "speed_control.h"
#include "heading_control.h"
#include "vehicle_yaw.h"

/* ---- 内部状态 ---- */
static uint8_t s_lastStartFlag = 0U;
static uint8_t s_running       = 0U;  /* 是否处于运行状态 */

void Mode1_Task(void)
{
    /* 首次调用时初始化：设置速度 20，启动控制器 */
    if (!SpeedControl_IsRunning()) {
        float refDeg;

        if (!VehicleYaw_IsCalibrated()) {
            return;
        }
        if (!VehicleYaw_GetStationaryReferenceDeg(&refDeg)) {
            return;
        }
        if (!SpeedControl_SetTargets(20, 20)) {
            return;
        }
        if (!SpeedControl_Start()) {
            return;
        }
        if (!HeadingControl_Start(refDeg)) {
            SpeedControl_Stop();
            return;
        }
    }
    /* 后续每 5ms 调用：不做额外操作，槽 2 中的速度环/航向环自动运行 */
}

void Mode2_Task(void)
{
    /* TODO: MODE2 具体逻辑 */
}

void Mode3_Task(void)
{
    /* TODO: MODE3 具体逻辑 */
}

void Mode4_Task(void)
{
    /* TODO: MODE4 具体逻辑 */
}

/*
 * 在 TIMER_0 ISR 槽 3 中每 5ms 调用一次。
 * 处理 RESET（急停）和 START（启动）标志位。
 */
void ModeExecTask(void)
{
    /* ---- RESET 优先：立即急停 ---- */
    if (g_menuResetFlag) {
        HeadingControl_Stop();
        SpeedControl_Stop();
        g_menuStartFlag = 0U;
        g_menuResetFlag = 0U;
        s_running       = 0U;
        return;
    }

    /* ---- START 上升沿：启动 ---- */
    if (g_menuStartFlag && !s_lastStartFlag) {
        if (g_menuMode >= 1U && g_menuMode <= 4U) {
            s_running = 1U;
        }
    }
    s_lastStartFlag = g_menuStartFlag;

    /* ---- 停止则返回 ---- */
    if (!g_menuStartFlag || !s_running) {
        return;
    }

    /* ---- 按模式分发 ---- */
    switch (g_menuMode) {
        case 1U: Mode1_Task(); break;
        case 2U: Mode2_Task(); break;
        case 3U: Mode3_Task(); break;
        case 4U: Mode4_Task(); break;
        default: break;
    }
}
