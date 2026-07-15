#include "key.h"
#include "clock.h"

/* 避免主循环长时间卡住后一次性补太大的时间，影响按键滤波手感。 */
#define KEY_MAX_UPDATE_MS   (50U)

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
} Key_PinCfg;

/* 按键表顺序必须和 Key_Id 枚举一致。 */
static const Key_PinCfg s_keyPins[KEY_COUNT] = {
    {KEY_KEY1_PORT, KEY_KEY1_PIN},
    {KEY_KEY2_PORT, KEY_KEY2_PIN},
    {KEY_KEY3_PORT, KEY_KEY3_PIN},
    {KEY_KEY4_PORT, KEY_KEY4_PIN},
};

/* 未滤波的原始按下状态，bit0=KEY1；1 表示 GPIO 当前读到按下。 */
static volatile uint8_t s_rawMask;

/* 候选状态：原始状态先稳定在这里，达到消抖时间后才更新到 s_pressedMask。 */
static uint8_t s_candidateMask;

/* 消抖后的稳定按下状态，bit0=KEY1。 */
static volatile uint8_t s_pressedMask;

/* 每个按键候选状态已经连续保持的时间，单位 ms。 */
static uint16_t s_debounceMs[KEY_COUNT];

/* 每个按键当前稳定按下的时间，单位 ms；松开后清零。 */
static volatile uint16_t s_pressedTimeMs[KEY_COUNT];

/* 上一次按键模块完成刷新的系统毫秒数。 */
static volatile unsigned long s_lastUpdateMs;

/* 防止主循环读取和定时器中断同时进入刷新函数。 */
static volatile bool s_updating;

/*
 * 读取 4 个 GPIO 的原始状态，并转换为按键位图。
 * 按键为内部上拉、低电平有效，所以 GPIO 读到 0 表示按下。
 */
static uint8_t Key_ReadRawMask(void)
{
    uint8_t mask = 0U;

    for (uint8_t i = 0U; i < KEY_COUNT; i++) {
        if (DL_GPIO_readPins(s_keyPins[i].port, s_keyPins[i].pin) == 0U) {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}

static uint16_t Key_LimitElapsedMs(unsigned long elapsedMs)
{
    if (elapsedMs > KEY_MAX_UPDATE_MS) {
        return KEY_MAX_UPDATE_MS;
    }

    return (uint16_t)elapsedMs;
}

static void Key_AddPressedTime(uint8_t key, uint16_t elapsedMs)
{
    uint32_t nextTime = (uint32_t)s_pressedTimeMs[key] + elapsedMs;

    s_pressedTimeMs[key] = (nextTime > UINT16_MAX) ? UINT16_MAX : (uint16_t)nextTime;
}

static void Key_ProcessElapsedMs(uint16_t elapsedMs)
{
    uint8_t rawMask = Key_ReadRawMask();
    s_rawMask = rawMask;

    if (elapsedMs == 0U) {
        return;
    }

    for (uint8_t i = 0U; i < KEY_COUNT; i++) {
        uint8_t bit = (uint8_t)(1U << i);
        bool rawPressed = (rawMask & bit) != 0U;
        bool candidatePressed = (s_candidateMask & bit) != 0U;
        bool stablePressed = (s_pressedMask & bit) != 0U;

        if (rawPressed != candidatePressed) {
            /* 原始状态刚变化，先作为候选状态记录下来，消抖计时重新开始。 */
            if (rawPressed) {
                s_candidateMask |= bit;
            } else {
                s_candidateMask &= (uint8_t)~bit;
            }
            s_debounceMs[i] = 0U;
        } else if (candidatePressed != stablePressed) {
            /* 候选状态和稳定状态不同，连续保持足够久后才确认变化。 */
            s_debounceMs[i] += elapsedMs;

            if (s_debounceMs[i] >= KEY_DEBOUNCE_MS) {
                if (candidatePressed) {
                    s_pressedMask |= bit;
                } else {
                    s_pressedMask &= (uint8_t)~bit;
                    s_pressedTimeMs[i] = 0U;
                }
                s_debounceMs[i] = 0U;
            }
        } else {
            s_debounceMs[i] = 0U;
        }

        if ((s_pressedMask & bit) != 0U) {
            Key_AddPressedTime(i, elapsedMs);
        } else {
            s_pressedTimeMs[i] = 0U;
        }
    }
}

/*
 * 按系统 tick 刷新一次按键状态。
 * 同一个 tick_ms 只处理一次，避免多个 getter 在同一轮 OLED 刷新里重复采集。
 */
static void Key_Service(void)
{
    unsigned long nowMs;
    unsigned long elapsedMs;

    if (s_updating) {
        return;
    }

    s_updating = true;
    nowMs = tick_ms;
    elapsedMs = nowMs - s_lastUpdateMs;

    if (elapsedMs > 0UL) {
        Key_ProcessElapsedMs(Key_LimitElapsedMs(elapsedMs));
        s_lastUpdateMs = nowMs;
    }

    s_updating = false;
}

void Key_Init(void)
{
    s_rawMask = Key_ReadRawMask();
    s_candidateMask = s_rawMask;
    s_pressedMask = s_rawMask;
    s_lastUpdateMs = tick_ms;
    s_updating = false;

    for (uint8_t i = 0U; i < KEY_COUNT; i++) {
        s_debounceMs[i] = 0U;
        s_pressedTimeMs[i] = 0U;
    }
}

void Key_Update1ms(void)
{
    Key_Service();
}

bool Key_IsPressed(Key_Id key)
{
    Key_Service();

    return ((uint8_t)key < KEY_COUNT) &&
           ((s_pressedMask & (uint8_t)(1U << (uint8_t)key)) != 0U);
}

uint8_t Key_GetRawPressedMask(void)
{
    Key_Service();

    return s_rawMask;
}

uint8_t Key_GetPressedMask(void)
{
    Key_Service();

    return s_pressedMask;
}

uint16_t Key_GetPressedTimeMs(Key_Id key)
{
    Key_Service();

    if ((uint8_t)key >= KEY_COUNT) {
        return 0U;
    }

    return s_pressedTimeMs[(uint8_t)key];
}