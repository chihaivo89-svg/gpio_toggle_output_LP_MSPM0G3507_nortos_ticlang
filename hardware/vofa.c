#include "vofa.h"

#include "clock.h"
#include "ti_msp_dl_config.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#define VOFA_SEND_PERIOD_MS    (40UL)
#define VOFA_FRAME_SIZE        (96U)

static unsigned long s_lastSendMs;
static volatile char s_rxCommand[VOFA_COMMAND_MAX_LENGTH];
static volatile uint32_t s_rxLength;
static volatile bool s_rxReady;

/* 通过 XDS110 独立串口发送 ASCII FireWater 帧。 */
void VOFA_SendMessage(const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(
            UART_VOFA_INST,
            (uint8_t)*text);
        text++;
    }
}

/* VOFA 接收中断只负责收集一行，命令解析留在主循环中完成。 */
void UART_VOFA_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_VOFA_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_VOFA_INST)) {
                uint8_t byte = DL_UART_Main_receiveData(UART_VOFA_INST);

                if (s_rxReady) {
                    continue;
                }

                if (byte == '\r' || byte == '\n') {
                    if (s_rxLength > 0U) {
                        s_rxCommand[s_rxLength] = '\0';
                        s_rxReady = true;
                    }
                } else if (s_rxLength < (VOFA_COMMAND_MAX_LENGTH - 1U)) {
                    s_rxCommand[s_rxLength++] = (char)byte;
                } else {
                    /* 超长命令直接丢弃，等待下一行重新接收。 */
                    s_rxLength = 0U;
                }
            }
            break;
        default:
            break;
    }
}

void VOFA_Init(void)
{
    s_lastSendMs = tick_ms;
    s_rxLength = 0U;
    s_rxReady = false;

    /*
     * UART commands arrive faster than the 1 ms IMU task can finish.  Let the
     * short RX interrupt preempt that timer so command bytes are not dropped.
     */
    NVIC_SetPriority(UART_VOFA_INST_INT_IRQN, 0U);
    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 1U);
    DL_UART_Main_enableInterrupt(
        UART_VOFA_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_VOFA_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_VOFA_INST_INT_IRQN);
}

bool VOFA_ReadCommand(char *command, uint32_t commandSize)
{
    uint32_t index;
    uint32_t copyLength;

    if (command == NULL || commandSize == 0U) {
        return false;
    }

    NVIC_DisableIRQ(UART_VOFA_INST_INT_IRQN);
    if (!s_rxReady) {
        NVIC_EnableIRQ(UART_VOFA_INST_INT_IRQN);
        return false;
    }

    copyLength = s_rxLength;
    if (copyLength >= commandSize) {
        copyLength = commandSize - 1U;
    }
    for (index = 0U; index < copyLength; index++) {
        command[index] = s_rxCommand[index];
    }
    command[copyLength] = '\0';

    s_rxLength = 0U;
    s_rxReady = false;
    NVIC_EnableIRQ(UART_VOFA_INST_INT_IRQN);

    return true;
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
    VOFA_SendMessage(frame);
}
