/*
 * speed_diag.c - 后轮速度一致性诊断实现
 */

#include "speed_diag.h"

#include <stdio.h>

#include "encoder.h"
#include "motor.h"
#include "speed_control.h"
#include "speed_control_config.h"
#include "ti_msp_dl_config.h"

#define SPEED_DIAG_SAMPLE_PERIOD_MS        (20U)
#define SPEED_DIAG_SAMPLE_COUNT            (500U)
#define SPEED_DIAG_WARMUP_SAMPLES          (50U)
#define SPEED_DIAG_LOW_TARGET              (8)
#define SPEED_DIAG_REVERSE_TARGET          (-8)
#define SPEED_DIAG_OPEN_LOOP_PWM           (500)

typedef struct {
    int16_t leftActual;
    int16_t rightActual;
    int16_t leftControlActual;
    int16_t rightControlActual;
    int16_t leftOutput;
    int16_t rightOutput;
    int16_t leftPositiveEdges;
    int16_t leftNegativeEdges;
    int16_t rightPositiveEdges;
    int16_t rightNegativeEdges;
} SpeedDiagSample;

static volatile SpeedDiagSample s_samples[SPEED_DIAG_SAMPLE_COUNT];
static volatile uint16_t s_sampleCount;
static volatile bool s_collecting;
static volatile bool s_resultValid;
static volatile bool s_startRequested;
static volatile bool s_stopRequested;
static volatile bool s_dumpRequested;
static volatile bool s_statusRequested;
static volatile bool s_openLoopRequested;
static volatile bool s_openLoopActive;
static volatile bool s_resultOpenLoop;
static volatile int32_t s_requestedTarget = SPEED_DEFAULT_TARGET;
static int32_t s_activeTarget;
static int32_t s_resultTarget;

static void SpeedDiag_HandleRxByte(uint8_t byte);

static int16_t SpeedDiag_ClampToInt16(int32_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static int32_t SpeedDiag_Abs(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void SpeedDiag_SendText(const char *text)
{
    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_DIAG_INST, (uint8_t)*text);
        text++;
    }
}

static void SpeedDiag_SendStatus(void)
{
    char line[96];

    (void)snprintf(
        line,
        sizeof(line),
        "rear:status active=%u samples=%u valid=%u target=%d\r\n",
        s_collecting ? 1U : 0U,
        (unsigned int)s_sampleCount,
        s_resultValid ? 1U : 0U,
        (int)s_activeTarget);
    SpeedDiag_SendText(line);
}

static void SpeedDiag_DumpResult(void)
{
    char line[160];
    uint16_t count = s_sampleCount;
    uint16_t firstMeasured;
    uint16_t measuredCount;
    uint16_t withinOne = 0U;
    int32_t leftSum = 0;
    int32_t rightSum = 0;
    int32_t absDiffSum = 0;
    int32_t maxAbsDiff = 0;

    if (!s_resultValid || count == 0U) {
        SpeedDiag_SendText("rear:error no completed test data\r\n");
        return;
    }

    firstMeasured =
        (count > SPEED_DIAG_WARMUP_SAMPLES) ? SPEED_DIAG_WARMUP_SAMPLES : 0U;
    measuredCount = count - firstMeasured;

    (void)snprintf(
        line,
        sizeof(line),
        "rear:dump begin mode=%s samples=%u period=%ums warmup=%u target=%d\r\n",
        s_resultOpenLoop ? "open" : "closed",
        (unsigned int)count,
        SPEED_DIAG_SAMPLE_PERIOD_MS,
        (unsigned int)firstMeasured,
        (int)s_resultTarget);
    SpeedDiag_SendText(line);
    SpeedDiag_SendText(
        "rear:columns index,m3_raw,m1_raw,raw_diff\r\n"
        "ctrl:columns index,m3_control,m3_pwm,m1_control,m1_pwm\r\n"
        "edge:columns index,m3_positive,m3_negative,m1_positive,m1_negative\r\n");

    for (uint16_t i = 0U; i < count; i++) {
        int32_t left = s_samples[i].leftActual;
        int32_t right = s_samples[i].rightActual;
        int32_t diff = left - right;

        (void)snprintf(
            line,
            sizeof(line),
            "rear:%u,%ld,%ld,%ld\r\n",
            (unsigned int)i,
            (long)left,
            (long)right,
            (long)diff);
        SpeedDiag_SendText(line);

        (void)snprintf(
            line,
            sizeof(line),
            "ctrl:%u,%d,%d,%d,%d\r\n",
            (unsigned int)i,
            (int)s_samples[i].leftControlActual,
            (int)s_samples[i].leftOutput,
            (int)s_samples[i].rightControlActual,
            (int)s_samples[i].rightOutput);
        SpeedDiag_SendText(line);

        (void)snprintf(
            line,
            sizeof(line),
            "edge:%u,%d,%d,%d,%d\r\n",
            (unsigned int)i,
            (int)s_samples[i].leftPositiveEdges,
            (int)s_samples[i].leftNegativeEdges,
            (int)s_samples[i].rightPositiveEdges,
            (int)s_samples[i].rightNegativeEdges);
        SpeedDiag_SendText(line);

        if (i >= firstMeasured) {
            int32_t absDiff = SpeedDiag_Abs(diff);

            leftSum += left;
            rightSum += right;
            absDiffSum += absDiff;
            if (absDiff > maxAbsDiff) {
                maxAbsDiff = absDiff;
            }
            if (absDiff <= 1) {
                withinOne++;
            }
        }
    }

    (void)snprintf(
        line,
        sizeof(line),
        "rear:summary n=%u m3_avg_x100=%ld m1_avg_x100=%ld "
        "signed_diff_x100=%ld abs_diff_x100=%ld max_diff=%ld within1=%u\r\n",
        (unsigned int)measuredCount,
        (long)((leftSum * 100) / measuredCount),
        (long)((rightSum * 100) / measuredCount),
        (long)(((leftSum - rightSum) * 100) / measuredCount),
        (long)((absDiffSum * 100) / measuredCount),
        (long)maxAbsDiff,
        (unsigned int)withinOne);
    SpeedDiag_SendText(line);
    SpeedDiag_SendText("rear:dump end\r\n");
}

static void SpeedDiag_HandleRxByte(uint8_t byte)
{
    switch (byte) {
        case 'T':
            s_requestedTarget = SPEED_DEFAULT_TARGET;
            s_startRequested = true;
            break;

        case 'L':
            s_requestedTarget = SPEED_DIAG_LOW_TARGET;
            s_startRequested = true;
            break;

        case 'R':
            s_requestedTarget = SPEED_DIAG_REVERSE_TARGET;
            s_startRequested = true;
            break;

        case 'O':
            s_openLoopRequested = true;
            break;

        case 'S':
            s_stopRequested = true;
            break;

        case 'D':
            s_dumpRequested = true;
            break;

        case '?':
            s_statusRequested = true;
            break;

        default:
            break;
    }
}

void UART_DIAG_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_DIAG_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_DIAG_INST)) {
                SpeedDiag_HandleRxByte(
                    DL_UART_Main_receiveData(UART_DIAG_INST));
            }
            break;

        default:
            break;
    }
}

void SpeedDiag_Init(void)
{
    /* 控制定时器和编码器优先于诊断串口，避免测试工具影响速度采样。 */
    NVIC_SetPriority(UART_DIAG_INST_INT_IRQN, 2U);
    DL_UART_Main_enableInterrupt(
        UART_DIAG_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_DIAG_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_DIAG_INST_INT_IRQN);
}

void SpeedDiag_Process(void)
{
    if (s_stopRequested) {
        s_stopRequested = false;
        s_collecting = false;
        s_resultValid = (s_sampleCount > 0U);
        SpeedControl_Stop();
        s_openLoopActive = false;
        SpeedDiag_SendText("rear:stopped\r\n");
    }

    if (s_startRequested) {
        char line[80];
        int32_t requestedTarget = s_requestedTarget;

        s_startRequested = false;

        if (s_collecting) {
            SpeedDiag_SendText("rear:error test already active\r\n");
        } else {
            SpeedControl_Stop();
            if (!SpeedControl_SetTargets(
                    requestedTarget, requestedTarget)) {
                SpeedDiag_SendText("rear:error invalid target\r\n");
            } else {
                (void)snprintf(
                    line,
                    sizeof(line),
                    "rear:test start target=%d samples=%u period=%ums\r\n",
                    (int)requestedTarget,
                    SPEED_DIAG_SAMPLE_COUNT,
                    SPEED_DIAG_SAMPLE_PERIOD_MS);
                SpeedDiag_SendText(line);

                s_sampleCount = 0U;
                s_resultValid = false;
                s_resultOpenLoop = false;
                s_openLoopActive = false;
                s_activeTarget = requestedTarget;
                s_resultTarget = requestedTarget;
                s_collecting = true;
                if (!SpeedControl_Start()) {
                    s_collecting = false;
                    SpeedDiag_SendText("rear:error speed control start failed\r\n");
                }
            }
        }
    }

    if (s_openLoopRequested) {
        char line[96];

        s_openLoopRequested = false;

        if (s_collecting) {
            SpeedDiag_SendText("rear:error test already active\r\n");
        } else {
            SpeedControl_Stop();
            s_sampleCount = 0U;
            s_resultValid = false;
            s_resultOpenLoop = true;
            s_openLoopActive = true;
            s_activeTarget = 0;
            s_resultTarget = 0;
            s_collecting = true;

            Motor_SetSpeed(MOTOR3, SPEED_DIAG_OPEN_LOOP_PWM);
            Motor_SetSpeed(MOTOR1, SPEED_DIAG_OPEN_LOOP_PWM);

            (void)snprintf(
                line,
                sizeof(line),
                "rear:open start pwm=%d samples=%u period=%ums\r\n",
                SPEED_DIAG_OPEN_LOOP_PWM,
                SPEED_DIAG_SAMPLE_COUNT,
                SPEED_DIAG_SAMPLE_PERIOD_MS);
            SpeedDiag_SendText(line);
        }
    }

    if (s_statusRequested) {
        s_statusRequested = false;
        SpeedDiag_SendStatus();
    }

    if (s_dumpRequested) {
        s_dumpRequested = false;
        SpeedDiag_DumpResult();
    }
}

void SpeedDiag_Record20ms(int32_t leftActual, int32_t rightActual)
{
    SpeedControlTelemetry telemetry;
    uint16_t index;

    if (!s_collecting) {
        return;
    }

    index = s_sampleCount;
    if (index >= SPEED_DIAG_SAMPLE_COUNT) {
        return;
    }

    if (s_openLoopActive) {
        telemetry.leftActual = leftActual;
        telemetry.rightActual = rightActual;
        telemetry.leftOutput = SPEED_DIAG_OPEN_LOOP_PWM;
        telemetry.rightOutput = SPEED_DIAG_OPEN_LOOP_PWM;
    } else {
        SpeedControl_GetTelemetry(&telemetry);
    }
    s_samples[index].leftActual = SpeedDiag_ClampToInt16(leftActual);
    s_samples[index].rightActual = SpeedDiag_ClampToInt16(rightActual);
    s_samples[index].leftControlActual =
        SpeedDiag_ClampToInt16(telemetry.leftActual);
    s_samples[index].rightControlActual =
        SpeedDiag_ClampToInt16(telemetry.rightActual);
    s_samples[index].leftOutput =
        SpeedDiag_ClampToInt16(telemetry.leftOutput);
    s_samples[index].rightOutput =
        SpeedDiag_ClampToInt16(telemetry.rightOutput);
    s_samples[index].leftPositiveEdges = SpeedDiag_ClampToInt16(
        (int32_t)Encoder_GetPositiveEdges(&gEncMotor3));
    s_samples[index].leftNegativeEdges = SpeedDiag_ClampToInt16(
        (int32_t)Encoder_GetNegativeEdges(&gEncMotor3));
    s_samples[index].rightPositiveEdges = SpeedDiag_ClampToInt16(
        (int32_t)Encoder_GetPositiveEdges(&gEncMotor1));
    s_samples[index].rightNegativeEdges = SpeedDiag_ClampToInt16(
        (int32_t)Encoder_GetNegativeEdges(&gEncMotor1));
    index++;
    s_sampleCount = index;

    if (index >= SPEED_DIAG_SAMPLE_COUNT) {
        s_collecting = false;
        s_resultValid = true;
        SpeedControl_Stop();
        s_openLoopActive = false;
        s_dumpRequested = true;
    }
}

bool SpeedDiag_IsActive(void)
{
    return s_collecting || s_startRequested ||
           s_openLoopRequested || s_dumpRequested;
}
