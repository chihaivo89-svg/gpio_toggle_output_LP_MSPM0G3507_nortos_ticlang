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
    int16_t leftActual;
    int16_t leftOutput;
    int16_t rightActual;
    int16_t rightOutput;
} RoadTest_Sample;

typedef struct {
    int16_t target;
    int16_t outputLimit;
    uint16_t sampleCount;
    int32_t leftActualSum;
    int32_t rightActualSum;
    int32_t leftErrorSum;
    int32_t rightErrorSum;
    int32_t sideDifferenceSum;
    uint16_t leftSaturationCount;
    uint16_t rightSaturationCount;
    RoadTest_Sample samples[ROAD_TEST_MAX_SAMPLES];
} RoadTest_RunLog;

/*
 * 每个样本只保存左右速度和左右 PWM，目标值及限幅保存在该次测试的头部。
 * 6 次测试共使用约 12KB SRAM，避免每次测试后都必须立即连接电脑。
 */
static volatile RoadTest_RunLog s_runLogs[ROAD_TEST_BATCH_RUN_COUNT];
static volatile uint8_t s_completedRunCount;
static volatile uint8_t s_activeRunIndex;

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

static int16_t RoadTest_TargetForRun(uint8_t runIndex)
{
    return (runIndex < ROAD_TEST_RUNS_PER_TARGET) ?
        ROAD_TEST_TARGET_8_PULSES : ROAD_TEST_TARGET_10_PULSES;
}

static int16_t RoadTest_LimitForRun(uint8_t runIndex)
{
    return (runIndex < ROAD_TEST_RUNS_PER_TARGET) ?
        ROAD_TEST_TARGET_8_LIMIT : ROAD_TEST_TARGET_10_LIMIT;
}

static uint8_t RoadTest_RunNumberWithinTarget(uint8_t runIndex)
{
    return (uint8_t)((runIndex % ROAD_TEST_RUNS_PER_TARGET) + 1U);
}

static volatile RoadTest_RunLog *RoadTest_GetActiveLog(void)
{
    if (s_activeRunIndex >= ROAD_TEST_BATCH_RUN_COUNT) {
        return NULL;
    }
    return &s_runLogs[s_activeRunIndex];
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

static void RoadTest_ResetRunLog(uint8_t runIndex)
{
    volatile RoadTest_RunLog *log;

    if (runIndex >= ROAD_TEST_BATCH_RUN_COUNT) {
        return;
    }

    log = &s_runLogs[runIndex];
    log->target = RoadTest_TargetForRun(runIndex);
    log->outputLimit = RoadTest_LimitForRun(runIndex);
    log->sampleCount = 0U;
    log->leftActualSum = 0;
    log->rightActualSum = 0;
    log->leftErrorSum = 0;
    log->rightErrorSum = 0;
    log->sideDifferenceSum = 0;
    log->leftSaturationCount = 0U;
    log->rightSaturationCount = 0U;
}

static void RoadTest_ResetBatch(void)
{
    uint8_t runIndex;

    for (runIndex = 0U;
         runIndex < ROAD_TEST_BATCH_RUN_COUNT;
         runIndex++) {
        RoadTest_ResetRunLog(runIndex);
    }

    s_completedRunCount = 0U;
    s_activeRunIndex = 0U;
}

static void RoadTest_SetState(RoadTest_State state)
{
    s_state = state;
    s_stateStartMs = tick_ms;
    s_oledNeedsClear = true;
}

static void RoadTest_BeginCountdown(void)
{
    if (s_completedRunCount >= ROAD_TEST_BATCH_RUN_COUNT) {
        return;
    }

    /* 如果此前由 VOFA 启动过电机，先无条件停车再准备新测试。 */
    (void)RoadTest_RunCommand("stop");
    s_activeRunIndex = s_completedRunCount;
    RoadTest_ResetRunLog(s_activeRunIndex);
    s_testAborted = false;
    RoadTest_SetState(ROAD_TEST_COUNTDOWN);
}

static void RoadTest_BeginRun(void)
{
    char command[24];
    bool configured;
    volatile RoadTest_RunLog *log = RoadTest_GetActiveLog();

    if (log == NULL) {
        s_testAborted = true;
        RoadTest_SetState(ROAD_TEST_RESULT);
        return;
    }

    /* 左右两侧均使用正目标值，因此 M1~M4 都向小车前进方向转动。 */
    configured = RoadTest_RunCommand("side=b");
    if (configured) {
        (void)snprintf(command, sizeof(command), "target=%d",
            (int)log->target);
        configured = RoadTest_RunCommand(command);
    }
    if (configured) {
        (void)snprintf(command, sizeof(command), "limit=%d",
            (int)log->outputLimit);
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
    volatile RoadTest_RunLog *log = RoadTest_GetActiveLog();

    /* 正常结束和人工中止都再次执行停车，形成独立的安全出口。 */
    (void)RoadTest_RunCommand("stop");

    /*
     * 只有完整运行且确实采到数据才计入批次；中途停止后本次编号不变，
     * 下一次启动会覆盖这组未完成数据。
     */
    s_testAborted = aborted || log == NULL || log->sampleCount == 0U;
    if (!s_testAborted &&
        s_completedRunCount < ROAD_TEST_BATCH_RUN_COUNT) {
        s_completedRunCount++;
    }

    RoadTest_SetState(ROAD_TEST_RESULT);
}

static void RoadTest_RenderCountdown(unsigned long nowMs)
{
    char line[24];
    volatile RoadTest_RunLog *log = RoadTest_GetActiveLog();
    unsigned long elapsed = nowMs - s_stateStartMs;
    unsigned long remaining = (elapsed < ROAD_TEST_COUNTDOWN_MS) ?
        (ROAD_TEST_COUNTDOWN_MS - elapsed) : 0UL;
    unsigned long seconds = (remaining + 999UL) / 1000UL;

    if (log == NULL) {
        return;
    }

    (void)snprintf(line, sizeof(line), "T%d RUN %u/3 RDY",
        (int)log->target,
        (unsigned int)RoadTest_RunNumberWithinTarget(s_activeRunIndex));
    RoadTest_ShowLine(0, line);
    (void)snprintf(line, sizeof(line), "START IN:%lus", seconds);
    RoadTest_ShowLine(1, line);
    (void)snprintf(line, sizeof(line), "TARGET:%d/20ms",
        (int)log->target);
    RoadTest_ShowLine(2, line);
    (void)snprintf(line, sizeof(line), "LIMIT:%d/1000",
        (int)log->outputLimit);
    RoadTest_ShowLine(3, line);
    (void)snprintf(line, sizeof(line), "RUN TIME:%lus",
        ROAD_TEST_DURATION_MS / 1000UL);
    RoadTest_ShowLine(4, line);
    (void)snprintf(line, sizeof(line), "SAVED:%u/%u",
        (unsigned int)s_completedRunCount,
        (unsigned int)ROAD_TEST_BATCH_RUN_COUNT);
    RoadTest_ShowLine(5, line);
    RoadTest_ShowLine(6, "KEEP PATH CLEAR");
    RoadTest_ShowLine(7, "K1:CANCEL");
}

static void RoadTest_RenderRunning(
    unsigned long nowMs,
    const SpeedControl_Status *speedStatus)
{
    char line[24];
    volatile RoadTest_RunLog *log = RoadTest_GetActiveLog();
    unsigned long elapsed = nowMs - s_stateStartMs;

    if (log == NULL) {
        return;
    }

    (void)snprintf(line, sizeof(line), "T%d RUN %u/3",
        (int)log->target,
        (unsigned int)RoadTest_RunNumberWithinTarget(s_activeRunIndex));
    RoadTest_ShowLine(0, line);
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
        (unsigned int)log->sampleCount);
    RoadTest_ShowLine(6, line);
    RoadTest_ShowLine(7, "K1:STOP");
}

static void RoadTest_RenderResult(void)
{
    char line[24];
    volatile RoadTest_RunLog *log = RoadTest_GetActiveLog();
    uint16_t count = (log != NULL) ? log->sampleCount : 0U;

    if (log == NULL) {
        return;
    }

    (void)snprintf(line, sizeof(line), "T%d RUN %u/3 %s",
        (int)log->target,
        (unsigned int)RoadTest_RunNumberWithinTarget(s_activeRunIndex),
        s_testAborted ? "STOP" : "DONE");
    RoadTest_ShowLine(0, line);

    if (count == 0U) {
        RoadTest_ShowLine(1, "NO SAMPLE DATA");
        RoadTest_ShowLine(2, "CHECK CONFIG");
        RoadTest_ShowLine(3, "");
        RoadTest_ShowLine(4, "");
        RoadTest_ShowLine(5, "");
        RoadTest_ShowLine(6, "");
    } else {
        RoadTest_ShowTenths(1, "L AVG:",
            RoadTest_AverageTenths(log->leftActualSum, count));
        RoadTest_ShowTenths(2, "R AVG:",
            RoadTest_AverageTenths(log->rightActualSum, count));
        RoadTest_ShowTenths(3, "L ERR:",
            RoadTest_AverageTenths(log->leftErrorSum, count));
        RoadTest_ShowTenths(4, "R ERR:",
            RoadTest_AverageTenths(log->rightErrorSum, count));
        RoadTest_ShowTenths(5, "SIDE D:",
            RoadTest_AverageTenths(log->sideDifferenceSum, count));
        (void)snprintf(line, sizeof(line), "SAT L:%u R:%u",
            (unsigned int)log->leftSaturationCount,
            (unsigned int)log->rightSaturationCount);
        RoadTest_ShowLine(6, line);
    }

    if (s_testAborted) {
        RoadTest_ShowLine(7, "K1:RETRY");
    } else if (s_completedRunCount >= ROAD_TEST_BATCH_RUN_COUNT) {
        RoadTest_ShowLine(7, "BATCH DONE");
    } else {
        uint8_t nextRunIndex = s_completedRunCount;

        (void)snprintf(line, sizeof(line), "K1:NEXT T%d-%u",
            (int)RoadTest_TargetForRun(nextRunIndex),
            (unsigned int)RoadTest_RunNumberWithinTarget(nextRunIndex));
        RoadTest_ShowLine(7, line);
    }
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
    RoadTest_ResetBatch();
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
            if (key1PressedEdge &&
                s_completedRunCount < ROAD_TEST_BATCH_RUN_COUNT) {
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
    volatile RoadTest_RunLog *log;
    volatile RoadTest_Sample *sample;
    uint16_t index;
    int32_t leftError;
    int32_t rightError;

    if (speedStatus == NULL || s_state != ROAD_TEST_RUNNING ||
        !speedStatus->running) {
        return;
    }

    log = RoadTest_GetActiveLog();
    if (log == NULL) {
        return;
    }

    index = log->sampleCount;
    if (index >= ROAD_TEST_MAX_SAMPLES) {
        return;
    }

    sample = &log->samples[index];
    sample->leftActual = RoadTest_ToInt16(speedStatus->leftActual);
    sample->leftOutput = RoadTest_ToInt16(speedStatus->leftOutput);
    sample->rightActual = RoadTest_ToInt16(speedStatus->rightActual);
    sample->rightOutput = RoadTest_ToInt16(speedStatus->rightOutput);

    leftError = RoadTest_Abs(speedStatus->leftTarget - speedStatus->leftActual);
    rightError = RoadTest_Abs(speedStatus->rightTarget - speedStatus->rightActual);
    log->leftActualSum += speedStatus->leftActual;
    log->rightActualSum += speedStatus->rightActual;
    log->leftErrorSum += leftError;
    log->rightErrorSum += rightError;
    log->sideDifferenceSum += RoadTest_Abs(
        speedStatus->leftActual - speedStatus->rightActual);

    if (RoadTest_Abs(speedStatus->leftOutput) >= log->outputLimit) {
        log->leftSaturationCount++;
    }
    if (RoadTest_Abs(speedStatus->rightOutput) >= log->outputLimit) {
        log->rightSaturationCount++;
    }

    /* 最后更新数量，主循环只能看到已经完整写入的样本。 */
    log->sampleCount = index + 1U;
}

bool RoadTest_UsesOled(void)
{
    return s_state != ROAD_TEST_IDLE;
}

void RoadTest_DumpToVofa(void)
{
    char frame[96];
    uint8_t runIndex;
    uint16_t index;
    uint16_t count;
    volatile const RoadTest_RunLog *log;

    if (s_state == ROAD_TEST_COUNTDOWN || s_state == ROAD_TEST_RUNNING) {
        VOFA_SendMessage("msg:ERR stop road test before dump\r\n");
        return;
    }
    if (s_completedRunCount == 0U) {
        VOFA_SendMessage("msg:ERR no road test data\r\n");
        return;
    }

    runIndex = (uint8_t)(s_completedRunCount - 1U);
    log = &s_runLogs[runIndex];
    count = log->sampleCount;

    (void)snprintf(frame, sizeof(frame),
        "msg:dump begin index=%u target=%d samples=%u period=20ms\r\n",
        (unsigned int)(runIndex + 1U),
        (int)log->target,
        (unsigned int)count);
    VOFA_SendMessage(frame);

    for (index = 0U; index < count; index++) {
        unsigned long frameStartMs = tick_ms;
        volatile const RoadTest_Sample *sample = &log->samples[index];

        (void)snprintf(frame, sizeof(frame),
            "pid:%d,%d,%d,%d,%d,%d\r\n",
            (int)log->target,
            (int)sample->leftActual,
            (int)sample->leftOutput,
            (int)log->target,
            (int)sample->rightActual,
            (int)sample->rightOutput);
        VOFA_SendMessage(frame);

        /* 按原始 20ms 节拍回放，VOFA 横轴和实测过程保持一致。 */
        while ((tick_ms - frameStartMs) < ROAD_TEST_SAMPLE_PERIOD_MS) {
        }
    }

    VOFA_SendMessage("msg:dump end\r\n");
}

static uint32_t RoadTest_ChecksumValue(uint32_t checksum, int16_t value)
{
    checksum ^= (uint16_t)value;
    return checksum * 16777619UL;
}

void RoadTest_DumpAllToVofa(void)
{
    char frame[112];
    uint8_t runIndex;
    uint8_t runCount = s_completedRunCount;

    if (s_state == ROAD_TEST_COUNTDOWN || s_state == ROAD_TEST_RUNNING) {
        VOFA_SendMessage("msg:ERR stop road test before dumpall\r\n");
        return;
    }
    if (runCount == 0U) {
        VOFA_SendMessage("msg:ERR no road test batch data\r\n");
        return;
    }

    (void)snprintf(frame, sizeof(frame),
        "msg:dumpall begin version=1 runs=%u expected=%u period=20ms\r\n",
        (unsigned int)runCount,
        (unsigned int)ROAD_TEST_BATCH_RUN_COUNT);
    VOFA_SendMessage(frame);

    for (runIndex = 0U; runIndex < runCount; runIndex++) {
        volatile const RoadTest_RunLog *log = &s_runLogs[runIndex];
        uint16_t sampleIndex;
        uint16_t count = log->sampleCount;
        uint32_t checksum = 2166136261UL;

        (void)snprintf(frame, sizeof(frame),
            "msg:run begin index=%u target=%d limit=%d samples=%u\r\n",
            (unsigned int)(runIndex + 1U),
            (int)log->target,
            (int)log->outputLimit,
            (unsigned int)count);
        VOFA_SendMessage(frame);

        for (sampleIndex = 0U; sampleIndex < count; sampleIndex++) {
            volatile const RoadTest_Sample *sample =
                &log->samples[sampleIndex];
            int16_t leftActual = sample->leftActual;
            int16_t leftOutput = sample->leftOutput;
            int16_t rightActual = sample->rightActual;
            int16_t rightOutput = sample->rightOutput;

            checksum = RoadTest_ChecksumValue(checksum, leftActual);
            checksum = RoadTest_ChecksumValue(checksum, leftOutput);
            checksum = RoadTest_ChecksumValue(checksum, rightActual);
            checksum = RoadTest_ChecksumValue(checksum, rightOutput);

            (void)snprintf(frame, sizeof(frame),
                "road:%u,%u,%d,%d,%d,%d\r\n",
                (unsigned int)(runIndex + 1U),
                (unsigned int)sampleIndex,
                (int)leftActual,
                (int)leftOutput,
                (int)rightActual,
                (int)rightOutput);
            VOFA_SendMessage(frame);
        }

        (void)snprintf(frame, sizeof(frame),
            "msg:run end index=%u checksum=%lu\r\n",
            (unsigned int)(runIndex + 1U),
            (unsigned long)checksum);
        VOFA_SendMessage(frame);
    }

    (void)snprintf(frame, sizeof(frame),
        "msg:dumpall end runs=%u\r\n",
        (unsigned int)runCount);
    VOFA_SendMessage(frame);
}

static const char *RoadTest_StateName(void)
{
    if (s_state == ROAD_TEST_COUNTDOWN) {
        return "countdown";
    }
    if (s_state == ROAD_TEST_RUNNING) {
        return "running";
    }
    if (s_state == ROAD_TEST_RESULT) {
        return (s_completedRunCount >= ROAD_TEST_BATCH_RUN_COUNT) ?
            "complete" : "result";
    }
    return "idle";
}

bool RoadTest_ProcessCommand(
    const char *command,
    char *reply,
    uint32_t replySize)
{
    volatile RoadTest_RunLog *log;

    if (command == NULL || reply == NULL || replySize == 0U) {
        return false;
    }

    if (strcmp(command, "roadstart") == 0) {
        if (s_state == ROAD_TEST_COUNTDOWN ||
            s_state == ROAD_TEST_RUNNING) {
            (void)snprintf(reply, replySize,
                "msg:ERR road test already active\r\n");
            return true;
        }
        if (s_completedRunCount >= ROAD_TEST_BATCH_RUN_COUNT) {
            (void)snprintf(reply, replySize,
                "msg:ERR batch complete; use dumpall or roadreset\r\n");
            return true;
        }

        RoadTest_BeginCountdown();
        log = RoadTest_GetActiveLog();
        (void)snprintf(reply, replySize,
            "msg:OK roadstart target=%d run=%u/3 countdown=2s\r\n",
            (int)log->target,
            (unsigned int)RoadTest_RunNumberWithinTarget(s_activeRunIndex));
        return true;
    }

    if (strcmp(command, "roadstop") == 0) {
        if (s_state == ROAD_TEST_RUNNING) {
            RoadTest_Finish(true);
            (void)snprintf(reply, replySize,
                "msg:OK road test stopped; run will be retried\r\n");
        } else if (s_state == ROAD_TEST_COUNTDOWN) {
            (void)RoadTest_RunCommand("stop");
            s_testAborted = true;
            RoadTest_SetState(ROAD_TEST_RESULT);
            (void)snprintf(reply, replySize,
                "msg:OK countdown cancelled; run will be retried\r\n");
        } else {
            (void)snprintf(reply, replySize,
                "msg:ERR no active road test\r\n");
        }
        return true;
    }

    if (strcmp(command, "roadreset") == 0) {
        (void)RoadTest_RunCommand("stop");
        RoadTest_ResetBatch();
        s_state = ROAD_TEST_IDLE;
        s_stateStartMs = tick_ms;
        s_testAborted = false;
        s_oledNeedsClear = false;
        s_key1WasPressed = Key_IsPressed(KEY_ID_1);
        OLED_Clear();
        (void)snprintf(reply, replySize,
            "msg:OK road batch reset; next target=8 run=1/3\r\n");
        return true;
    }

    if (strcmp(command, "roadstatus") == 0) {
        log = RoadTest_GetActiveLog();
        (void)snprintf(reply, replySize,
            "msg:road state=%s saved=%u/%u active=%u "
            "target=%d run=%u/3 samples=%u\r\n",
            RoadTest_StateName(),
            (unsigned int)s_completedRunCount,
            (unsigned int)ROAD_TEST_BATCH_RUN_COUNT,
            (unsigned int)(s_activeRunIndex + 1U),
            (log != NULL) ? (int)log->target : 0,
            (unsigned int)RoadTest_RunNumberWithinTarget(s_activeRunIndex),
            (log != NULL) ? (unsigned int)log->sampleCount : 0U);
        return true;
    }

    if (strcmp(command, "roadhelp") == 0) {
        (void)snprintf(reply, replySize,
            "msg:roadstart roadstop roadstatus roadreset dump dumpall\r\n");
        return true;
    }

    return false;
}
