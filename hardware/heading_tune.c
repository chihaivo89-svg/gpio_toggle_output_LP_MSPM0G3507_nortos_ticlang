#include "heading_tune.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "heading_control.h"
#include "speed_control.h"
#include "speed_control_config.h"
#include "ti_msp_dl_config.h"

/* This module is temporary test infrastructure, not the final vehicle UI. */
#define HEADING_TUNE_SAMPLE_PERIOD_MS       (20U)
#define HEADING_TUNE_MAX_SAMPLES             (250U)
#define HEADING_TUNE_MIN_DURATION_MS        (1000U)
#define HEADING_TUNE_MAX_DURATION_MS        (5000U)
#define HEADING_TUNE_MIN_TARGET                (8)
#define HEADING_TUNE_MAX_TARGET               (20)
#define HEADING_TUNE_RX_BUFFER_SIZE           (64U)

typedef struct {
    int16_t headingDeciDeg;
    int16_t errorDeciDeg;
    int16_t rateDeciDps;
    int16_t targetOffset;
    int16_t leftTarget;
    int16_t rightTarget;
    int16_t leftActual;
    int16_t rightActual;
    int16_t leftOutput;
    int16_t rightOutput;
} HeadingTuneSample;

static volatile HeadingTuneSample s_samples[HEADING_TUNE_MAX_SAMPLES];
static volatile HeadingTuneSample s_liveSample;

static volatile char s_rxBuffer[HEADING_TUNE_RX_BUFFER_SIZE];
static volatile uint8_t s_rxLength;
static volatile bool s_rxCollecting;
static volatile bool s_commandReady;

static volatile bool s_armRequested;
static volatile bool s_remoteRunRequested;
static volatile bool s_stopRequested;
static volatile bool s_dumpRequested;
static volatile bool s_completePending;
static volatile bool s_livePending;
static volatile bool s_collecting;
static volatile uint16_t s_sampleCount;

static bool s_armed;
static bool s_resultValid;
static bool s_liveEnabled;
static bool s_liveStreaming;
static int32_t s_target;
static uint16_t s_durationMs;
static uint16_t s_requestedSampleCount;
static int32_t s_resultTarget;
static uint16_t s_resultDurationMs;
static int16_t s_resultReferenceDeciDeg;
static int16_t s_resultKpMilli;
static int16_t s_resultKdMilli;
static int16_t s_resultDeadbandDeciDeg;
static int16_t s_resultMaxOffset;

static int16_t HeadingTune_ClampInt16(int32_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static int16_t HeadingTune_FloatToScaled(float value, float scale)
{
    float scaled = value * scale;
    int32_t rounded = (scaled >= 0.0f) ?
        (int32_t)(scaled + 0.5f) : (int32_t)(scaled - 0.5f);

    return HeadingTune_ClampInt16(rounded);
}

static void HeadingTune_SendText(const char *text)
{
    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(UART_DIAG_INST, (uint8_t)*text);
        text++;
    }
}

static bool HeadingTune_ParseInt32(const char *text, int32_t *value)
{
    bool negative = false;
    bool hasDigit = false;
    int32_t result = 0;

    if (text == 0 || value == 0 || *text == '\0') {
        return false;
    }

    if (*text == '-' || *text == '+') {
        negative = (*text == '-');
        text++;
    }

    while (*text != '\0') {
        if (*text < '0' || *text > '9') {
            return false;
        }
        hasDigit = true;
        if (result > 1000000) {
            return false;
        }
        result = result * 10 + (int32_t)(*text - '0');
        text++;
    }

    if (!hasDigit) {
        return false;
    }

    *value = negative ? -result : result;
    return true;
}

static bool HeadingTune_ParseFloat(const char *text, float *value)
{
    bool negative = false;
    bool hasDigit = false;
    int32_t integerPart = 0;
    int32_t fractionPart = 0;
    int32_t fractionScale = 1;
    float result;

    if (text == 0 || value == 0 || *text == '\0') {
        return false;
    }

    if (*text == '-' || *text == '+') {
        negative = (*text == '-');
        text++;
    }

    while (*text >= '0' && *text <= '9') {
        hasDigit = true;
        if (integerPart > 1000000) {
            return false;
        }
        integerPart = integerPart * 10 + (int32_t)(*text - '0');
        text++;
    }

    if (*text == '.') {
        text++;
        while (*text >= '0' && *text <= '9') {
            hasDigit = true;
            if (fractionScale < 1000000) {
                fractionPart = fractionPart * 10 + (int32_t)(*text - '0');
                fractionScale *= 10;
            }
            text++;
        }
    }

    if (!hasDigit || *text != '\0') {
        return false;
    }

    result = (float)integerPart +
             (float)fractionPart / (float)fractionScale;
    *value = negative ? -result : result;
    return true;
}

static bool HeadingTune_GetArgument(
    const char *command,
    const char *name,
    const char **argument)
{
    size_t length = strlen(name);

    if (strncmp(command, name, length) == 0 &&
        command[length] == ',' && command[length + 1U] != '\0') {
        *argument = &command[length + 1U];
        return true;
    }
    return false;
}

static void HeadingTune_SendStatus(void)
{
    HeadingControlTelemetry telemetry;
    char line[240];

    HeadingControl_GetTelemetry(&telemetry);
    (void)snprintf(
        line,
        sizeof(line),
        "msg:head armed=%u active=%u result=%u samples=%u target=%ld "
        "duration=%u live=%u kp=%.3f kd=%.3f db=%.1f maxoff=%ld "
        "ref=%+.1f hdg=%+.1f err=%+.1f cmd=%ld\r\n",
        s_armed ? 1U : 0U,
        s_collecting ? 1U : 0U,
        s_resultValid ? 1U : 0U,
        (unsigned int)s_sampleCount,
        (long)s_target,
        (unsigned int)s_durationMs,
        s_liveEnabled ? 1U : 0U,
        telemetry.kp,
        telemetry.kd,
        telemetry.deadbandDeg,
        (long)telemetry.maxTargetOffset,
        telemetry.referenceDeg,
        telemetry.headingDeg,
        telemetry.errorDeg,
        (long)telemetry.targetOffset);
    HeadingTune_SendText(line);
}

static void HeadingTune_SendHelp(void)
{
    HeadingTune_SendText(
        "msg:head commands STATUS KP,x KD,x DB,x OFFSET,n TARGET,n "
        "DURATION,ms LIVE,0|1 ARM RUN STOP DISARM DUMP\r\n");
}

static void HeadingTune_DumpResult(void)
{
    char line[200];
    uint16_t count = s_sampleCount;

    if (!s_resultValid || count == 0U) {
        HeadingTune_SendText("msg:head error no completed heading test data\r\n");
        return;
    }

    (void)snprintf(
        line,
        sizeof(line),
        "head:dump begin samples=%u period=%ums target=%ld duration=%u "
        "ref_x10=%d kp_x1000=%d kd_x1000=%d db_x10=%d maxoff=%d\r\n",
        (unsigned int)count,
        HEADING_TUNE_SAMPLE_PERIOD_MS,
        (long)s_resultTarget,
        (unsigned int)s_resultDurationMs,
        (int)s_resultReferenceDeciDeg,
        (int)s_resultKpMilli,
        (int)s_resultKdMilli,
        (int)s_resultDeadbandDeciDeg,
        (int)s_resultMaxOffset);
    HeadingTune_SendText(line);
    HeadingTune_SendText(
        "head:columns index,heading_x10,error_x10,rate_x10,offset,"
        "left_target,right_target,left_actual,right_actual,left_pwm,right_pwm\r\n");

    for (uint16_t index = 0U; index < count; index++) {
        const volatile HeadingTuneSample *sample = &s_samples[index];

        (void)snprintf(
            line,
            sizeof(line),
            "head:%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
            (unsigned int)index,
            (int)sample->headingDeciDeg,
            (int)sample->errorDeciDeg,
            (int)sample->rateDeciDps,
            (int)sample->targetOffset,
            (int)sample->leftTarget,
            (int)sample->rightTarget,
            (int)sample->leftActual,
            (int)sample->rightActual,
            (int)sample->leftOutput,
            (int)sample->rightOutput);
        HeadingTune_SendText(line);
    }

    HeadingTune_SendText("head:dump end\r\n");
}

static void HeadingTune_SendLiveSample(const volatile HeadingTuneSample *sample)
{
    char line[176];

    (void)snprintf(
        line,
        sizeof(line),
        "head:%+.1f,%+.1f,%+.1f,%d,%d,%d,%d,%d,%d,%d\r\n",
        (double)sample->headingDeciDeg / 10.0,
        (double)sample->errorDeciDeg / 10.0,
        (double)sample->rateDeciDps / 10.0,
        (int)sample->targetOffset,
        (int)sample->leftTarget,
        (int)sample->rightTarget,
        (int)sample->leftActual,
        (int)sample->rightActual,
        (int)sample->leftOutput,
        (int)sample->rightOutput);
    HeadingTune_SendText(line);
}

static void HeadingTune_ProcessCommand(const char *command)
{
    const char *argument;
    float floatValue;
    int32_t intValue;
    char line[96];

    if (strncmp(command, "H,", 2U) != 0) {
        HeadingTune_SendText("msg:head error invalid frame\r\n");
        return;
    }

    command += 2;
    if (strcmp(command, "STATUS") == 0) {
        HeadingTune_SendStatus();
    } else if (strcmp(command, "HELP") == 0) {
        HeadingTune_SendHelp();
    } else if (strcmp(command, "ARM") == 0) {
        s_armRequested = true;
    } else if (strcmp(command, "RUN") == 0) {
        s_remoteRunRequested = true;
    } else if (strcmp(command, "STOP") == 0) {
        s_stopRequested = true;
    } else if (strcmp(command, "DISARM") == 0) {
        HeadingTune_Disarm();
    } else if (strcmp(command, "DUMP") == 0) {
        s_dumpRequested = true;
    } else if (HeadingTune_GetArgument(command, "KP", &argument)) {
        if (HeadingTune_ParseFloat(argument, &floatValue) &&
            HeadingControl_SetKp(floatValue)) {
            (void)snprintf(line, sizeof(line),
                           "msg:head ok kp=%.3f\r\n", floatValue);
            HeadingTune_SendText(line);
        } else {
            HeadingTune_SendText("msg:head error kp out of range\r\n");
        }
    } else if (HeadingTune_GetArgument(command, "KD", &argument)) {
        if (HeadingTune_ParseFloat(argument, &floatValue) &&
            HeadingControl_SetKd(floatValue)) {
            (void)snprintf(line, sizeof(line),
                           "msg:head ok kd=%.3f\r\n", floatValue);
            HeadingTune_SendText(line);
        } else {
            HeadingTune_SendText("msg:head error kd out of range\r\n");
        }
    } else if (HeadingTune_GetArgument(command, "DB", &argument)) {
        if (HeadingTune_ParseFloat(argument, &floatValue) &&
            HeadingControl_SetDeadbandDeg(floatValue)) {
            (void)snprintf(line, sizeof(line),
                           "msg:head ok db=%.1f\r\n", floatValue);
            HeadingTune_SendText(line);
        } else {
            HeadingTune_SendText("msg:head error db out of range\r\n");
        }
    } else if (HeadingTune_GetArgument(command, "OFFSET", &argument)) {
        if (HeadingTune_ParseInt32(argument, &intValue) &&
            HeadingControl_SetMaxTargetOffset(intValue)) {
            (void)snprintf(line, sizeof(line),
                           "msg:head ok maxoff=%ld\r\n", (long)intValue);
            HeadingTune_SendText(line);
        } else {
            HeadingTune_SendText("msg:head error offset out of range\r\n");
        }
    } else if (HeadingTune_GetArgument(command, "TARGET", &argument)) {
        if (HeadingTune_ParseInt32(argument, &intValue) &&
            intValue >= HEADING_TUNE_MIN_TARGET &&
            intValue <= HEADING_TUNE_MAX_TARGET) {
            s_target = intValue;
            (void)snprintf(line, sizeof(line),
                           "msg:head ok target=%ld\r\n", (long)s_target);
            HeadingTune_SendText(line);
        } else {
            HeadingTune_SendText("msg:head error target must be 8..20\r\n");
        }
    } else if (HeadingTune_GetArgument(command, "DURATION", &argument)) {
        if (HeadingTune_ParseInt32(argument, &intValue) &&
            intValue >= HEADING_TUNE_MIN_DURATION_MS &&
            intValue <= HEADING_TUNE_MAX_DURATION_MS) {
            s_durationMs = (uint16_t)intValue;
            (void)snprintf(line, sizeof(line),
                           "msg:head ok duration=%u\r\n",
                           (unsigned int)s_durationMs);
            HeadingTune_SendText(line);
        } else {
            HeadingTune_SendText("msg:head error duration must be 1000..5000\r\n");
        }
    } else if (HeadingTune_GetArgument(command, "LIVE", &argument)) {
        if (HeadingTune_ParseInt32(argument, &intValue) &&
            (intValue == 0 || intValue == 1)) {
            s_liveEnabled = (intValue == 1);
            (void)snprintf(line, sizeof(line),
                           "msg:head ok live=%ld\r\n", (long)intValue);
            HeadingTune_SendText(line);
        } else {
            HeadingTune_SendText("msg:head error live must be 0 or 1\r\n");
        }
    } else {
        HeadingTune_SendText("msg:head error unknown command\r\n");
    }
}

void HeadingTune_Init(void)
{
    s_rxLength = 0U;
    s_rxCollecting = false;
    s_commandReady = false;
    s_armRequested = false;
    s_remoteRunRequested = false;
    s_stopRequested = false;
    s_dumpRequested = false;
    s_completePending = false;
    s_livePending = false;
    s_collecting = false;
    s_sampleCount = 0U;
    s_armed = false;
    s_resultValid = false;
    s_liveEnabled = true;
    s_liveStreaming = false;
    s_target = SPEED_DEFAULT_TARGET;
    s_durationMs = HEADING_TUNE_MAX_DURATION_MS;
    s_requestedSampleCount = HEADING_TUNE_MAX_SAMPLES;
    s_resultTarget = 0;
    s_resultDurationMs = 0U;
    s_resultReferenceDeciDeg = 0;
    s_resultKpMilli = 0;
    s_resultKdMilli = 0;
    s_resultDeadbandDeciDeg = 0;
    s_resultMaxOffset = 0;
}

bool HeadingTune_HandleRxByte(uint8_t byte)
{
    if (byte == '@') {
        if (s_commandReady) {
            return true;
        }
        s_rxLength = 0U;
        s_rxCollecting = true;
        return true;
    }

    if (!s_rxCollecting) {
        return false;
    }

    if (byte == '\r') {
        return true;
    }

    if (byte == '\n') {
        if (s_rxLength > 0U) {
            s_rxBuffer[s_rxLength] = '\0';
            s_commandReady = true;
        }
        s_rxCollecting = false;
        return true;
    }

    if (s_rxLength >= (HEADING_TUNE_RX_BUFFER_SIZE - 1U)) {
        s_rxLength = 0U;
        s_rxCollecting = false;
        return true;
    }

    s_rxBuffer[s_rxLength] = (char)byte;
    s_rxLength++;
    return true;
}

void HeadingTune_Process(void)
{
    char command[HEADING_TUNE_RX_BUFFER_SIZE];
    uint8_t length;

    if (s_commandReady) {
        length = s_rxLength;
        if (length >= HEADING_TUNE_RX_BUFFER_SIZE) {
            length = HEADING_TUNE_RX_BUFFER_SIZE - 1U;
        }
        for (uint8_t index = 0U; index < length; index++) {
            command[index] = (char)s_rxBuffer[index];
        }
        command[length] = '\0';
        s_commandReady = false;
        HeadingTune_ProcessCommand(command);
    }

    if (s_completePending) {
        char line[112];

        s_completePending = false;
        (void)snprintf(
            line,
            sizeof(line),
            "msg:head complete samples=%u target=%ld; send @H,DUMP\r\n",
            (unsigned int)s_sampleCount,
            (long)s_resultTarget);
        HeadingTune_SendText(line);
    }

    if (s_dumpRequested) {
        s_dumpRequested = false;
        HeadingTune_DumpResult();
    }

    if (s_livePending) {
        s_livePending = false;
        HeadingTune_SendLiveSample(&s_liveSample);
    }
}

bool HeadingTune_TakeArmRequest(void)
{
    bool requested = s_armRequested;

    s_armRequested = false;
    return requested;
}

bool HeadingTune_TakeRemoteRunRequest(void)
{
    bool requested = s_remoteRunRequested;

    s_remoteRunRequested = false;
    return requested;
}

bool HeadingTune_TakeStopRequest(void)
{
    bool requested = s_stopRequested;

    s_stopRequested = false;
    return requested;
}

void HeadingTune_Arm(void)
{
    char line[112];

    s_armed = true;
    (void)snprintf(
        line,
        sizeof(line),
        "msg:head armed target=%ld duration=%u; disconnect then press KEY1\r\n",
        (long)s_target,
        (unsigned int)s_durationMs);
    HeadingTune_SendText(line);
}

void HeadingTune_Disarm(void)
{
    s_armed = false;
    HeadingTune_SendText("msg:head disarmed\r\n");
}

bool HeadingTune_IsArmed(void)
{
    return s_armed;
}

bool HeadingTune_IsActive(void)
{
    return s_collecting;
}

int32_t HeadingTune_GetTarget(void)
{
    return s_target;
}

bool HeadingTune_BeginRun(bool remoteStart)
{
    HeadingControlTelemetry telemetry;
    char line[144];
    uint32_t samples;

    if (s_collecting || !SpeedControl_IsRunning() ||
        !HeadingControl_IsActive()) {
        return false;
    }

    samples = ((uint32_t)s_durationMs + HEADING_TUNE_SAMPLE_PERIOD_MS - 1U) /
              HEADING_TUNE_SAMPLE_PERIOD_MS;
    if (samples > HEADING_TUNE_MAX_SAMPLES) {
        samples = HEADING_TUNE_MAX_SAMPLES;
    }
    if (samples == 0U) {
        return false;
    }

    HeadingControl_GetTelemetry(&telemetry);
    s_armed = false;
    s_collecting = true;
    s_resultValid = false;
    s_sampleCount = 0U;
    s_requestedSampleCount = (uint16_t)samples;
    s_resultTarget = s_target;
    s_resultDurationMs = s_durationMs;
    s_resultReferenceDeciDeg = HeadingTune_FloatToScaled(
        telemetry.referenceDeg, 10.0f);
    s_resultKpMilli = HeadingTune_FloatToScaled(telemetry.kp, 1000.0f);
    s_resultKdMilli = HeadingTune_FloatToScaled(telemetry.kd, 1000.0f);
    s_resultDeadbandDeciDeg = HeadingTune_FloatToScaled(
        telemetry.deadbandDeg, 10.0f);
    s_resultMaxOffset = HeadingTune_ClampInt16(telemetry.maxTargetOffset);
    s_liveStreaming = s_liveEnabled && remoteStart;
    s_livePending = false;
    s_completePending = false;

    (void)snprintf(
        line,
        sizeof(line),
        "msg:head run source=%s target=%ld samples=%u period=%ums live=%u\r\n",
        remoteStart ? "uart" : "key1",
        (long)s_target,
        (unsigned int)s_requestedSampleCount,
        HEADING_TUNE_SAMPLE_PERIOD_MS,
        s_liveStreaming ? 1U : 0U);
    HeadingTune_SendText(line);
    return true;
}

void HeadingTune_Abort(void)
{
    s_armed = false;
    s_collecting = false;
    s_liveStreaming = false;
    s_livePending = false;
    s_completePending = false;
    s_resultValid = false;
}

void HeadingTune_Record20ms(int32_t leftActual, int32_t rightActual)
{
    HeadingControlTelemetry heading;
    SpeedControlTelemetry speed;
    uint16_t index;

    if (!s_collecting) {
        return;
    }

    if (!SpeedControl_IsRunning() || !HeadingControl_IsActive()) {
        s_collecting = false;
        s_liveStreaming = false;
        s_resultValid = false;
        return;
    }

    index = s_sampleCount;
    if (index >= s_requestedSampleCount ||
        index >= HEADING_TUNE_MAX_SAMPLES) {
        return;
    }

    HeadingControl_GetTelemetry(&heading);
    SpeedControl_GetTelemetry(&speed);
    s_samples[index].headingDeciDeg = HeadingTune_FloatToScaled(
        heading.headingDeg, 10.0f);
    s_samples[index].errorDeciDeg = HeadingTune_FloatToScaled(
        heading.errorDeg, 10.0f);
    s_samples[index].rateDeciDps = HeadingTune_FloatToScaled(
        heading.yawRateDps, 10.0f);
    s_samples[index].targetOffset = HeadingTune_ClampInt16(
        heading.targetOffset);
    s_samples[index].leftTarget = HeadingTune_ClampInt16(speed.leftTarget);
    s_samples[index].rightTarget = HeadingTune_ClampInt16(speed.rightTarget);
    s_samples[index].leftActual = HeadingTune_ClampInt16(leftActual);
    s_samples[index].rightActual = HeadingTune_ClampInt16(rightActual);
    s_samples[index].leftOutput = HeadingTune_ClampInt16(speed.leftOutput);
    s_samples[index].rightOutput = HeadingTune_ClampInt16(speed.rightOutput);

    if (s_liveStreaming && (index & 1U) == 0U) {
        s_liveSample = s_samples[index];
        s_livePending = true;
    }

    index++;
    s_sampleCount = index;
    if (index >= s_requestedSampleCount) {
        s_collecting = false;
        s_liveStreaming = false;
        s_resultValid = true;
        s_completePending = true;
        HeadingControl_Stop();
        SpeedControl_Stop();
    }
}

void HeadingTune_ReportStartFailure(const char *reason)
{
    char line[112];

    (void)snprintf(
        line,
        sizeof(line),
        "msg:head error start failed: %s\r\n",
        (reason != 0) ? reason : "unknown");
    HeadingTune_SendText(line);
}
