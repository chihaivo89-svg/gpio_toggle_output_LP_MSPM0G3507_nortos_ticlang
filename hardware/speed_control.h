/*
 *  speed_control.h  —— 左右两侧速度闭环控制
 *
 *  左侧使用 M3 编码器反馈，同一 PID 输出驱动 M3/M4；
 *  右侧使用 M1 编码器反馈，同一 PID 输出驱动 M1/M2。
 *  M2/M4 没有独立速度反馈，作为同侧开环跟随电机。
 */

#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SPEED_CONTROL_LEFT = 0,
    SPEED_CONTROL_RIGHT,
    SPEED_CONTROL_BOTH
} SpeedControl_Mode;

typedef struct {
    int32_t leftTarget;
    int32_t leftActual;
    int32_t leftOutput;
    int32_t rightTarget;
    int32_t rightActual;
    int32_t rightOutput;
    bool running;
    SpeedControl_Mode mode;
} SpeedControl_Status;

/* 初始化后所有电机保持停止，必须收到 run 命令才会启动闭环。 */
void SpeedControl_Init(void);

/* 每获得一组新的 20ms 编码器数据时调用一次。 */
void SpeedControl_Update20ms(int32_t leftActual, int32_t rightActual);

/* 获取供 OLED/VOFA 使用的状态快照。 */
void SpeedControl_GetStatus(SpeedControl_Status *status);

/*
 * 处理 VOFA 文本命令，并把执行结果写入 reply。
 * 返回 true 表示命令有效，false 表示格式或参数错误。
 */
bool SpeedControl_ProcessCommand(
    const char *command,
    char *reply,
    uint32_t replySize);

#endif /* __SPEED_CONTROL_H */
