#include "vofa.h"

#include "clock.h"
#include "ti_msp_dl_config.h"

#include <stdio.h>
#include <stdint.h>

#define VOFA_SEND_PERIOD_MS    (40UL)
#define VOFA_FRAME_SIZE        (96U)

static unsigned long s_lastSendMs;

/* 通过 XDS110 独立串口发送 ASCII FireWater 帧。 */
static void VOFA_SendText(const char *text)
{
    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(
            UART_VOFA_INST,
            (uint8_t)*text);
        text++;
    }
}

void VOFA_Init(void)
{
    s_lastSendMs = tick_ms;
}

void VOFA_Update(const VOFA_SpeedData *data)
{
    char frame[VOFA_FRAME_SIZE];
    unsigned long nowMs = tick_ms;

    if (data == NULL) {
        return;
    }

    if ((nowMs - s_lastSendMs) < VOFA_SEND_PERIOD_MS) {
        return;
    }

    s_lastSendMs = nowMs;

    /* 顺序固定为 I0~I5，与 VOFA 三张速度 PID 波形图一一对应。 */
    (void)snprintf(
        frame,
        sizeof(frame),
        "pid:%ld,%ld,%ld,%ld,%ld,%ld\r\n",
        (long)data->leftTarget,
        (long)data->leftActual,
        (long)data->leftOutput,
        (long)data->rightTarget,
        (long)data->rightActual,
        (long)data->rightOutput);
    VOFA_SendText(frame);
}
