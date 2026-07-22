/*
 * key.h — 按键驱动模块（移植自 TC264）
 *
 * 消抖算法：每 5ms 采样一次 GPIO，5 次为一周期（25ms）。
 * ≥4/5 次读到低电平 → 确认按下。
 * 长按：稳定按下持续 ≥ KEY_LONG_PRESS_TICKS 个周期后触发单次事件。
 */

#ifndef __KEY_H
#define __KEY_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/* GPIO -- key -- GND, internal pull-up: released=1, pressed=0. */
#define KEY_COUNT          (4U)

/* 按键索引（1-based，与 TC264 一致） */
#define KEY_ID_1   (1u)
#define KEY_ID_2   (2u)
#define KEY_ID_3   (3u)
#define KEY_ID_4   (4u)

/*
 * 长按阈值，单位：去抖稳定周期（25ms）。
 * 默认 24 → 约 600ms。
 */
#ifndef KEY_LONG_PRESS_TICKS
#define KEY_LONG_PRESS_TICKS   (24u)
#endif

/* ---- 初始化 ---- */
void Key_Init(void);

/*
 * 每 1ms 调用一次（内部 5 分频，实际采样周期 5ms）。
 * 在 TIMER_0 的 1ms ISR 中调用。
 */
void Key_Update1ms(void);

/*
 * 读取指定按键稳定状态（去抖后）。
 * i: 1/2/3/4  → KEY1/KEY2/KEY3/KEY4
 * return: 1=按下, 0=松开
 */
uint8_t Key_Read(uint8_t i);

/*
 * 读取指定按键长按事件（单次触发，读取后自动清零）。
 * i: 1/2/3/4
 * return: 1=本次读取时触发长按, 0=未触发
 */
uint8_t Key_ReadLongPress(uint8_t i);

/*
 * 获取指定按键稳定按下的持续时间。
 * i: 1/2/3/4
 * return: 持续时间 ms；松开后归零。最大 65535ms。
 */
uint16_t Key_GetPressedTimeMs(uint8_t i);

#endif /* __KEY_H */
