#ifndef __KEY_H
#define __KEY_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * 按键硬件接法：GPIO -- 按键 -- GND。
 * SysConfig 中已经把 KEY1~KEY4 配置为内部上拉输入，
 * 所以松开时 GPIO 为高电平 1，按下时 GPIO 被拉到低电平 0。
 */
#define KEY_COUNT          (4U)

/* 按键状态连续稳定 KEY_DEBOUNCE_MS 后，才确认按下或松开。 */
#define KEY_DEBOUNCE_MS    (20U)

/*
 * 按键编号顺序和 SysConfig 中的 KEY1~KEY4 保持一致。
 * KEY_ID_1 -> KEY1 -> PA2
 * KEY_ID_2 -> KEY2 -> PB1
 * KEY_ID_3 -> KEY3 -> PB17
 * KEY_ID_4 -> KEY4 -> PB20
 */
typedef enum {
    KEY_ID_1 = 0,
    KEY_ID_2,
    KEY_ID_3,
    KEY_ID_4,
} Key_Id;

/* 初始化按键软件状态，在 SYSCFG_DL_init() 之后调用。 */
void Key_Init(void);

/* 按键刷新函数，可在 TIMER_0 的 1ms 中断中调用。 */
void Key_Update1ms(void);

/* 查询当前稳定状态：按住时持续返回 true，松开返回 false。 */
bool Key_IsPressed(Key_Id key);

/* 获取未滤波的原始按下状态位图：bit0=KEY1，bit1=KEY2，bit2=KEY3，bit3=KEY4。 */
uint8_t Key_GetRawPressedMask(void);

/* 获取当前稳定按下状态位图：bit0=KEY1，bit1=KEY2，bit2=KEY3，bit3=KEY4。 */
uint8_t Key_GetPressedMask(void);

/* 获取某个按键当前稳定按下的时间，单位 ms；松开后自动清零。 */
uint16_t Key_GetPressedTimeMs(Key_Id key);

#endif /* __KEY_H */