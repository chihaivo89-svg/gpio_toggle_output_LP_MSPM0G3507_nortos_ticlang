#include "road_test.h"

#include "clock.h"
#include "key.h"
#include "oled_hardware_i2c.h"
#include "vofa.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ROAD_TEST_MAX_SAMPLES       \
    ((ROAD_TEST_DURATION_MS / ROAD_TEST_SAMPLE_PERIOD_MS) + 10U)
#define ROAD_TEST_OLED_PERIOD_MS     (100UL)
#define ROAD_TEST_LINE_LENGTH        (16U)

typedef struct {
    int16_t leftTarget;
    int16_t leftActual;
    int16_t leftOutput;
    int16_t rightTarget;
    int16_t rightActual;
    int16_t rightOutput;
} RoadTest_Sample;

static RoadTest_Sample s_samples[ROAD_TEST_MAX_SAMPLES];
static volatile uint16_t s_sampleCount;
static volatile int32_t s_leftActualSum;
static volatile int32_t s_rightActualSum;
static volatile int32_t s_leftErrorSum;
static volatile int32_t s_rightErrorSum;
static volatile int32_t s_sideDifferenceSum;
static volatile uint16_t s_leftSaturationCount;
static volatile uint16_t s_rightSaturationCount;

static volatile RoadTest_State s_state;
static unsigned long s_stateStartMs;
static unsigned long s_lastOledMs;
static bool s_key1WasPressed;
static bool s_oledNeedsClear;
static bool s_testAborted;

static int32_t RoadTest_Abs(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int16_t RoadTest_ToInt16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32768) {
        return -32768;
    }
    return (int16_t)value;
}

/* OLED 每行固定写满 16 个字符，避免短字符串后面残留旧内容。 */
static void RoadTest_ShowLine(uint8_t row, const char *text)
{
    char line[ROAD_TEST_LINE_LENGTH + 1U];
    uint32_t index = 0U;

    while (index < ROAD_TEST_LINE_LENGTH && text[index] != '\0') {
        line[index] = text[index];
        index++;
    }
    while (index < ROAD_TEST_LINE_LENGTH) {
        line[index++] = ' ';
    }
    line[ROAD_TEST_LINE_LENGTH] = '\0';
    OLED_ShowString(0, row, (uint8_t *)line, 8);
}

static void RoadTest_ShowTenths(uint8_t row, const char *label, int32_t value)
{
    char line[24];
    uint32_t magnitude;

    if (value < 0) {
        magnitude = (uint32_t)(-value);
        (void)snprintf(line, sizeof(line), "%s-%lu.%lu", label,
            (unsigned long)(magnitude / 10U),
            (unsigned long)(magnitude % 10U));
    } else {
        magnitude = (uint32_t)value;
        (void)snprintf(line, sizeof(line), "%s%lu.%lu", label,
            (unsigned long)(magnitude / 10U),
            (unsigned long)(magnitude % 10U));
    }
    RoadTest_ShowLine(row, line);
}

static int32_t RoadTest_AverageTenths(int32_t sum, uint16_t count)
{
    int32_t scaled;

    if (count == 0U) {
        return 0;
    }

    scaled = sum * 10;
    if (scaled >= 0) {
        return (scaled + (int32_t)(count / 2U)) / (int32_t)count;
    }
    return (scaled - (int32_t)(count / 2U)) / (int32_t)count;
}

static bool RoadTest_RunCommand(const char *command)
{
    char reply[96];

    return SpeedControl_ProcessCommand(command, reply, sizeof(reply));
}

static void RoadTest_ResetLog(void)
{
    s_sampleCount = 0U;
    s_leftActualSum = 0;
    s_rightActualSum = 0;
    s_leftErrorSum = 0;
    s_rightErrorSum = 0;
    s_sideDifferenceSum = 0;
    s_leftSaturationCount = 0U;
    s_rightSaturationCount = 0U;
}

static void RoadTest_SetState(RoadTest_State state)
{
    s_state = state;
    s_stateStartMs = tick_ms;
    s_oledNeedsClear = true;
}

static void RoadTest_BeginCountdown(void)
{
    /* 如果此前由 VOFA 启动过电机，先无条件停车再准备新测试。 */
    (void)RoadTest_RunCommand("stop");
    RoadTest_ResetLog();
    s_testAborted = false;
    RoadTest_SetState(ROAD_TEST_COUNTDOWN);
}

static void RoadTest_BeginRun(void)
{
    char command[24];
    bool configured;

    /* 左右两侧均使用正目标值，因此 M1~M4 都向小车前进方向转动。 */
    configured = RoadTest_RunCommand("side=b");
    if (configured) {
        (void)snprintf(command, sizeof(command), "target=%d",
            ROAD_TEST_TARGET_PULSES);
        configured = RoadTest_RunCommand(command);
    }
    if (configured) {
        (void)snprintf(command, sizeof(command), "limit=%d",
            ROAD_TEST_OUTPUT_LIMIT);
        configured = RoadTest_RunCommand(command);
    }
    if (configured) {
        configured = RoadTest_RunCommand("run");
    }

    if (!configured) {
        (void)RoadTest_RunCommand("stop");
        s_testAborted = true;
        RoadTest_SetState(ROAD_TEST_RESULT);
        return;
    }

    RoadTest_SetState(ROAD_TEST_RUNNING);
}

static void RoadTest_Finish(bool aborted)
{
    /* 正常结束和人工中止都再次执行停车，形成独立的安全出口。 */
    (void)RoadTest_RunCommand("stop");
    s_testAborted = aborted;
    RoadTest_SetState(ROAD_TEST_RESULT);
}

static void RoadTest_RenderCountdown(unsigned long nowMs)
{
    char line[24];
    unsigned long elapsed = nowMs - s_stateStartMs;
    unsigned long remaining = (elapsed < ROAD_TEST_COUNTDOWN_MS) ?
        (ROAD_TEST_COUNTDOWN_MS - elapsed) : 0UL;
    unsigned long seconds = (remaining + 999UL) / 1000UL;

    RoadTest_ShowLine(0, "ROAD TEST READY");
    (void)snprintf(line, sizeof(line), "START IN:%lus", seconds);
    RoadTest_ShowLine(1, line);
    (void)snprintf(line, sizeof(line), "TARGET:%d/20ms",
        ROAD_TEST_TARGET_PULSES);
    RoadTest_ShowLine(2, line);
    (void)snprintf(line, sizeof(line), "LIMIT:%d/1000",
        ROAD_TEST_OUTPUT_LIMIT);
    RoadTest_ShowLine(3, line);
    (void)snprintf(line, sizeof(line), "RUN TIME:%lus",
        ROAD_TEST_DURATION_MS / 1000UL);
    RoadTest_ShowLine(4, line);
    RoadTest_ShowLine(5, "ALL WHEELS FWD");
    RoadTest_ShowLine(6, "KEEP PATH CLEAR");
    RoadTest_ShowLine(7, "K1:CANCEL");
}

static void RoadTest_RenderRunning(
    unsigned long nowMs,
    const SpeedControl_Status *speedStatus)
{
    char line[24];
    unsigned long elapsed = nowMs - s_stateStartMs;

    RoadTest_ShowLine(0, "ROAD TEST RUN");
    (void)snprintf(line, sizeof(line), "TIME:%lu.%lu/%lus",
        elapsed / 1000UL, (elapsed % 1000UL) / 100UL,
        ROAD_TEST_DURATION_MS / 1000UL);
    RoadTest_ShowLine(1, line);

    if (speedStatus != NULL) {
        (void)snprintf(line, sizeof(line), "L T:%ld A:%ld",
            (long)speedStatus->leftTarget,
            (long)speedStatus->leftActual);
        RoadTest_ShowLine(2, line);
        (void)snprintf(line, sizeof(line), "R T:%ld A:%ld",
            (long)speedStatus->rightTarget,
            (long)speedStatus->rightActual);
        RoadTest_ShowLine(3, line);
        (void)snprintf(line, sizeof(line), "PWM L:%ld",
            (long)speedStatus->leftOutput);
        RoadTest_ShowLine(4, line);
        (void)snprintf(line, sizeof(line), "PWM R:%ld",
            (long)speedStatus->rightOutput);
        RoadTest_ShowLine(5, line);
    }

    (void)snprintf(line, sizeof(line), "SAMPLES:%u",
        (unsigned int)s_sampleCount);
    RoadTest_ShowLine(6, line);
    RoadTest_ShowLine(7, "K1:STOP");
}

static void RoadTest_RenderResult(void)
{
    char line[24];
    uint16_t count = s_sampleCount;

    RoadTest_ShowLine(0, s_testAborted ? "ROAD TEST STOP" : "ROAD TEST DONE");

    if (count == 0U) {
        RoadTest_ShowLine(1, "NO SAMPLE DATA");
        RoadTest_ShowLine(2, "CHECK CONFIG");
        RoadTest_ShowLine(3, "");
        RoadTest_ShowLine(4, "");
        RoadTest_ShowLine(5, "");
        RoadTest_ShowLine(6, "");
    } else {
        RoadTest_ShowTenths(1, "L AVG:",
            RoadTest_AverageTenths(s_leftActualSum, count));
        RoadTest_ShowTenths(2, "R AVG:",
            RoadTest_AverageTenths(s_rightActualSum, count));
        RoadTest_ShowTenths(3, "L ERR:",
            RoadTest_AverageTenths(s_leftErrorSum, count));
        RoadTest_ShowTenths(4, "R ERR:",
            RoadTest_AverageTenths(s_rightErrorSum, count));
        RoadTest_ShowTenths(5, "SIDE D:",
            RoadTest_AverageTenths(s_sideDifferenceSum, count));
        (void)snprintf(line, sizeof(line), "SAT L:%u R:%u",
            (unsigned int)s_leftSaturationCount,
            (unsigned int)s_rightSaturationCount);
        RoadTest_ShowLine(6, line);
    }

    RoadTest_ShowLine(7, "K1:RETEST");
}

static void RoadTest_RenderOled(const SpeedControl_Status *speedStatus)
{
    unsigned long nowMs = tick_ms;
    bool force = s_oledNeedsClear;

    if (!force && (nowMs - s_lastOledMs) < ROAD_TEST_OLED_PERIOD_MS) {
        return;
    }

    if (force) {
        OLED_Clear();
        s_oledNeedsClear = false;
    }
    s_lastOledMs = nowMs;

    if (s_state == ROAD_TEST_COUNTDOWN) {
        RoadTest_RenderCountdown(nowMs);
    } else if (s_state == ROAD_TEST_RUNNING) {
        RoadTest_RenderRunning(nowMs, speedStatus);
    } else if (s_state == ROAD_TEST_RESULT) {
        RoadTest_RenderResult();
    }
}

void RoadTest_Init(void)
{
    s_state = ROAD_TEST_IDLE;
    s_stateStartMs = tick_ms;
    s_lastOledMs = tick_ms;
    s_key1WasPressed = Key_IsPressed(KEY_ID_1);
    s_oledNeedsClear = false;
    s_testAborted = false;
    RoadTest_ResetLog();
}

void RoadTest_Update(const SpeedControl_Status *speedStatus)
{
    bool key1Pressed = Key_IsPressed(KEY_ID_1);
    bool key1PressedEdge = key1Pressed && !s_key1WasPressed;
    unsigned long elapsed = tick_ms - s_stateStartMs;

    s_key1WasPressed = key1Pressed;

    switch (s_state) {
        case ROAD_TEST_IDLE:
            if (key1PressedEdge) {
                RoadTest_BeginCountdown();
            }
            break;

        case ROAD_TEST_COUNTDOWN:
            if (key1PressedEdge) {
                (void)RoadTest_RunCommand("stop");
                s_state = ROAD_TEST_IDLE;
                OLED_Clear();
            } else if (elapsed >= ROAD_TEST_COUNTDOWN_MS) {
                RoadTest_BeginRun();
            }
            break;

        case ROAD_TEST_RUNNING:
            if (key1PressedEdge) {
                RoadTest_Finish(true);
            } else if ((speedStatus != NULL && !speedStatus->running) ||
                       elapsed >= ROAD_TEST_DURATION_MS) {
                RoadTest_Finish(false);
            }
            break;

        case ROAD_TEST_RESULT:
            if (key1PressedEdge) {
                RoadTest_BeginCountdown();
            }
            break;

        default:
            (void)RoadTest_RunCommand("stop");
            s_state = ROAD_TEST_IDLE;
            break;
    }

    if (s_state != ROAD_TEST_IDLE) {
        RoadTest_RenderOled(speedStatus);
    }
}

void RoadTest_Record20ms(const SpeedControl_Status *speedStatus)
{
    uint16_t index;
    int32_t leftError;
    int32_t rightError;

    if (speedStatus == NULL || s_state != ROAD_TEST_RUNNING ||
        !speedStatus->running) {
        return;
    }

    index = s_sampleCount;
    if (index >= ROAD_TEST_MAX_SAMPLES) {
        return;
    }

    s_samples[index].leftTarget = RoadTest_ToInt16(speedStatus->leftTarget);
    s_samples[index].leftActual = RoadTest_ToInt16(speedStatus->leftActual);
    s_samples[index].leftOutput = RoadTest_ToInt16(speedStatus->leftOutput);
    s_samples[index].rightTarget = RoadTest_ToInt16(speedStatus->rightTarget);
    s_samples[index].rightActual = RoadTest_ToInt16(speedStatus->rightActual);
    s_samples[index].rightOutput = RoadTest_ToInt16(speedStatus->rightOutput);

    leftError = RoadTest_Abs(speedStatus->leftTarget - speedStatus->leftActual);
    rightError = RoadTest_Abs(speedStatus->rightTarget - speedStatus->rightActual);
    s_leftActualSum += speedStatus->leftActual;
    s_rightActualSum += speedStatus->rightActual;
    s_leftErrorSum += leftError;
    s_rightErrorSum += rightError;
    s_sideDifferenceSum += RoadTest_Abs(
        speedStatus->leftActual - speedStatus->rightActual);

    if (RoadTest_Abs(speedStatus->leftOutput) >= ROAD_TEST_OUTPUT_LIMIT) {
        s_leftSaturationCount++;
    }
    if (RoadTest_Abs(speedStatus->rightOutput) >= ROAD_TEST_OUTPUT_LIMIT) {
        s_rightSaturationCount++;
    }

    /* 最后更新数量，主循环只能看到已经完整写入的样本。 */
    s_sampleCount = index + 1U;
}

bool RoadTest_UsesOled(void)
{
    return s_state != ROAD_TEST_IDLE;
}

void RoadTest_DumpToVofa(void)
{
    char frame[96];
    uint16_t index;
    uint16_t count = s_sampleCount;

    if (s_state == ROAD_TEST_COUNTDOWN || s_state == ROAD_TEST_RUNNING) {
        VOFA_SendMessage("msg:ERR stop road test before dump\r\n");
        return;
    }
    if (count == 0U) {
        VOFA_SendMessage("msg:ERR no road test data\r\n");
        return;
    }

    (void)snprintf(frame, sizeof(frame),
        "msg:dump begin samples=%u period=20ms\r\n",
        (unsigned int)count);
    VOFA_SendMessage(frame);

    for (index = 0U; index < count; index++) {
        unsigned long frameStartMs = tick_ms;
        const RoadTest_Sample *sample = &s_samples[index];

        (void)snprintf(frame, sizeof(frame),
            "pid:%d,%d,%d,%d,%d,%d\r\n",
            (int)sample->leftTarget,
            (int)sample->leftActual,
            (int)sample->leftOutput,
            (int)sample->rightTarget,
            (int)sample->rightActual,
            (int)sample->rightOutput);
        VOFA_SendMessage(frame);

        /* 按原始 20ms 节拍回放，VOFA 横轴和实测过程保持一致。 */
        while ((tick_ms - frameStartMs) < ROAD_TEST_SAMPLE_PERIOD_MS) {
        }
    }

    VOFA_SendMessage("msg:dump end\r\n");
}
