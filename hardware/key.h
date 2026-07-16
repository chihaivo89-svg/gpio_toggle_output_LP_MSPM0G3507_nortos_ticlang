#ifndef __KEY_H
#define __KEY_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/* GPIO -- key -- GND, internal pull-up: released=1, pressed=0. */
#define KEY_COUNT          (4U)
#define KEY_DEBOUNCE_MS    (20U)

typedef enum {
    KEY_ID_1 = 0,    /* PA2  */
    KEY_ID_2,        /* PB1  */
    KEY_ID_3,        /* PB17 */
    KEY_ID_4,        /* PB20 */
} Key_Id;

void Key_Init(void);
void Key_Update1ms(void);
bool Key_IsPressed(Key_Id key);
/* 获取未滤波和完成 20ms 消抖后的四按键状态位图。 */
uint8_t Key_GetRawPressedMask(void);
uint8_t Key_GetPressedMask(void);
/* 获取某个按键稳定按下的持续时间；松开后返回 0。 */
uint16_t Key_GetPressedTimeMs(Key_Id key);

#endif /* __KEY_H */
