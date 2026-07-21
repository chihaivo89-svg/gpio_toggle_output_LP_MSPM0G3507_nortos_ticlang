/*
 * speed_control.h - 四轮小车左右两侧速度闭环
 *
 * 左侧：M3 编码器反馈，M3 闭环驱动，M4 同侧跟随。
 * 右侧：M1 编码器反馈，M1 闭环驱动，M2 同侧跟随。
 *
 * 参数调整入口：speed_control_config.h
 *
 * 调用顺序：
 *   1. SpeedControl_Init()
 *   2. SpeedControl_SetTargets()
 *   3. SpeedControl_Start()
 *   4. 每获得一组 20ms 编码器数据时调用 SpeedControl_Update20ms()
 *   5. SpeedControl_Stop() 立即停止四个电机
 */

#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/* 初始化正式速度环参数。初始化完成后四个电机保持停止。 */
void SpeedControl_Init(void);

/*
 * 设置左右侧目标速度，单位为每 20ms 的编码器脉冲数。
 * 目标范围为 -200~200；参数越界时保持原目标并返回 false。
 * 运行期间允许更新目标，内部斜坡会平滑跟随新目标。
 */
bool SpeedControl_SetTargets(int32_t leftTarget, int32_t rightTarget);

/*
 * 航向外环差速入口。正偏移表示 leftTarget+offset、
 * and rightTarget-offset, but only while both base targets are forward.
 * Start()、Stop() 和 SetTargets() 都会清零该偏移。
 */
void SpeedControl_SetDifferentialTargetOffsetFloat(float offset);

/* 至少一侧目标非零时启动，启动成功返回 true。 */
bool SpeedControl_Start(void);

/* 立即停止四个电机并清除 PID 历史状态。 */
void SpeedControl_Stop(void);

/* 返回速度环当前是否处于运行状态。 */
bool SpeedControl_IsRunning(void);

/* 每获得一组新的 20ms 编码器数据时调用一次。 */
void SpeedControl_Update20ms(int32_t leftActual, int32_t rightActual);

#endif /* __SPEED_CONTROL_H */
