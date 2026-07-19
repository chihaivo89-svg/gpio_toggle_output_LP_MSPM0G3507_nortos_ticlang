/*
 * speed_diag.h - 后轮速度一致性诊断
 *
 * 该模块只用于架空验证 M3（左后轮）与 M1（右后轮）的速度一致性。
 * 测试期间仅采集 RAM 数据，停车后才通过 UART1（PA8/PA9）批量输出，
 * 避免串口发送阻塞 20ms 速度环。
 *
 * 单字节命令：
 *   T - 以 SPEED_DEFAULT_TARGET 正转并采集 10 秒
 *   L - 以 Target 8 低速正转并采集 10 秒
 *   R - 以 Target -8 低速反转并采集 10 秒
 *   O - 以固定 PWM 500 开环运行并采集 10 秒
 *   S - 立即停止
 *   D - 重新输出最近一次结果
 *   ? - 查询诊断状态
 */

#ifndef SPEED_DIAG_H
#define SPEED_DIAG_H

#include <stdbool.h>
#include <stdint.h>

void SpeedDiag_Init(void);
void SpeedDiag_Process(void);
void SpeedDiag_Record20ms(int32_t leftActual, int32_t rightActual);
bool SpeedDiag_IsActive(void);

#endif /* SPEED_DIAG_H */
