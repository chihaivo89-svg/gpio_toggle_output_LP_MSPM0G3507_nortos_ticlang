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
bool Key_WasPressed(Key_Id key);
uint8_t Key_GetPressedMask(void);
uint8_t Key_GetPressedEventMask(void);

#endif /* __KEY_H */