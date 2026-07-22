/*
 * key.c — 按键驱动模块（移植自 TC264）
 *
 * 消抖算法：
 *   - 每 5ms 采样一次 GPIO，5 次采样为一周期（25ms）。
 *   - 周期内累计"按下次数"，≥4/5 次 → 确认按下。
 *   - 长按事件在稳定状态节拍上统计，达到阈值后触发单次事件。
 *
 * 移植说明：
 *   - 原 TC264 的 gpio_get_level() → 替换为 DL_GPIO_readPins()
 *   - 原 TC264 的 gpio_init() → 删除（引脚由 SysConfig 统一配置）
 *   - 新增按 1ms 累加的按下计时，供 OLED 显示
 */

#include "key.h"

/* ---- 引脚表（顺序与 KEY_ID_1..4 对应，索引 = KEY_ID-1） ---- */
typedef struct {
    GPIO_Regs *port;
    uint32_t   pin;
} Key_PinCfg;

static const Key_PinCfg s_keyPins[KEY_COUNT] = {
    {KEY_KEY1_PORT, KEY_KEY1_PIN},   /* KEY_ID_1 → index 0 */
    {KEY_KEY2_PORT, KEY_KEY2_PIN},   /* KEY_ID_2 → index 1 */
    {KEY_KEY3_PORT, KEY_KEY3_PIN},   /* KEY_ID_3 → index 2 */
    {KEY_KEY4_PORT, KEY_KEY4_PIN},   /* KEY_ID_4 → index 3 */
};

/* ---- 5 分频：Key_Update1ms() 每 1ms 调用，内部每 5ms 执行一次实际采样 ---- */
static uint8_t s_ms_divider = 0U;

/* ---- 5 次采样为一个周期 ---- */
static uint8_t s_sample_tick = 0U;       /* 0..4 */
static uint8_t s_k1_cnt = 0U;
static uint8_t s_k2_cnt = 0U;
static uint8_t s_k3_cnt = 0U;
static uint8_t s_k4_cnt = 0U;

/* ---- 去抖后的稳定状态：1=按下 ---- */
static uint8_t s_k1_down = 0U;
static uint8_t s_k2_down = 0U;
static uint8_t s_k3_down = 0U;
static uint8_t s_k4_down = 0U;

/* ---- 长按检测状态（单位：稳定采样周期，约 25ms） ---- */
static uint8_t s_k1_hold_ticks = 0U;
static uint8_t s_k2_hold_ticks = 0U;
static uint8_t s_k3_hold_ticks = 0U;
static uint8_t s_k4_hold_ticks = 0U;

static uint8_t s_k1_long_latched = 0U;
static uint8_t s_k2_long_latched = 0U;
static uint8_t s_k3_long_latched = 0U;
static uint8_t s_k4_long_latched = 0U;

static uint8_t s_k1_long_event = 0U;
static uint8_t s_k2_long_event = 0U;
static uint8_t s_k3_long_event = 0U;
static uint8_t s_k4_long_event = 0U;

/* ---- 按下计时（单位 ms）：供 OLED 显示 ---- */
static volatile uint16_t s_k1_pressed_ms = 0U;
static volatile uint16_t s_k2_pressed_ms = 0U;
static volatile uint16_t s_k3_pressed_ms = 0U;
static volatile uint16_t s_k4_pressed_ms = 0U;

/* ---- 内部辅助 ---- */

static uint8_t Key_ReadPin(uint8_t index)
{
    /* 低电平有效：读到 0 表示按下 */
    return (DL_GPIO_readPins(s_keyPins[index].port, s_keyPins[index].pin) == 0U) ? 0U : 1U;
}

static void key_long_press_update(uint8_t  down,
                                  uint8_t *hold_ticks,
                                  uint8_t *long_latched,
                                  uint8_t *long_event)
{
    if (down) {
        if (*hold_ticks < 255U) {
            (*hold_ticks)++;
        }

        if ((*long_latched == 0U) && (*hold_ticks >= KEY_LONG_PRESS_TICKS)) {
            *long_latched = 1U;
            *long_event   = 1U;
        }
    } else {
        *hold_ticks    = 0U;
        *long_latched  = 0U;
    }
}

/*
 * 实际采样一次（每 5ms 调用）。
 * 算法与 TC264 code/Key.c 的 Key_Sample() 完全一致。
 */
static void Key_Sample(void)
{
    /* KEYx 读到 0 表示按下：周期内累计"按下次数" */
    if (Key_ReadPin(0U) == 0U) { s_k1_cnt++; }
    if (Key_ReadPin(1U) == 0U) { s_k2_cnt++; }
    if (Key_ReadPin(2U) == 0U) { s_k3_cnt++; }
    if (Key_ReadPin(3U) == 0U) { s_k4_cnt++; }

    /* 5 次采样为一个周期：第 5 次采样后更新稳定状态 */
    s_sample_tick++;
    if (s_sample_tick >= 5U) {
        s_sample_tick = 0U;

        /* ≥4/5 认为按下（抗抖） */
        s_k1_down = (uint8_t)(s_k1_cnt >= 4U);
        s_k2_down = (uint8_t)(s_k2_cnt >= 4U);
        s_k3_down = (uint8_t)(s_k3_cnt >= 4U);
        s_k4_down = (uint8_t)(s_k4_cnt >= 4U);

        /* 长按事件在"稳定状态"节拍上统计 */
        key_long_press_update(s_k1_down, &s_k1_hold_ticks, &s_k1_long_latched, &s_k1_long_event);
        key_long_press_update(s_k2_down, &s_k2_hold_ticks, &s_k2_long_latched, &s_k2_long_event);
        key_long_press_update(s_k3_down, &s_k3_hold_ticks, &s_k3_long_latched, &s_k3_long_event);
        key_long_press_update(s_k4_down, &s_k4_hold_ticks, &s_k4_long_latched, &s_k4_long_event);

        /* 清零计数，进入下一周期 */
        s_k1_cnt = 0U;
        s_k2_cnt = 0U;
        s_k3_cnt = 0U;
        s_k4_cnt = 0U;
    }
}

static void Key_AccumPressedTime(void)
{
    if (s_k1_down) { if (s_k1_pressed_ms < 65535U) { s_k1_pressed_ms++; } } else { s_k1_pressed_ms = 0U; }
    if (s_k2_down) { if (s_k2_pressed_ms < 65535U) { s_k2_pressed_ms++; } } else { s_k2_pressed_ms = 0U; }
    if (s_k3_down) { if (s_k3_pressed_ms < 65535U) { s_k3_pressed_ms++; } } else { s_k3_pressed_ms = 0U; }
    if (s_k4_down) { if (s_k4_pressed_ms < 65535U) { s_k4_pressed_ms++; } } else { s_k4_pressed_ms = 0U; }
}

/* ---- 公共 API ---- */

void Key_Init(void)
{
    s_ms_divider   = 0U;
    s_sample_tick  = 0U;
    s_k1_cnt = 0U; s_k2_cnt = 0U; s_k3_cnt = 0U; s_k4_cnt = 0U;
    s_k1_down = 0U; s_k2_down = 0U; s_k3_down = 0U; s_k4_down = 0U;

    s_k1_hold_ticks = 0U; s_k2_hold_ticks = 0U;
    s_k3_hold_ticks = 0U; s_k4_hold_ticks = 0U;

    s_k1_long_latched = 0U; s_k2_long_latched = 0U;
    s_k3_long_latched = 0U; s_k4_long_latched = 0U;

    s_k1_long_event = 0U; s_k2_long_event = 0U;
    s_k3_long_event = 0U; s_k4_long_event = 0U;

    s_k1_pressed_ms = 0U; s_k2_pressed_ms = 0U;
    s_k3_pressed_ms = 0U; s_k4_pressed_ms = 0U;
}

/*
 * 由 TIMER_0 的 1ms ISR 调用。
 * 内部 5 分频：每 5ms 执行一次 Key_Sample()。
 * 同时每 1ms 累加按下计时供 OLED 显示。
 */
void Key_Update1ms(void)
{
    s_ms_divider++;
    if (s_ms_divider >= 5U) {
        s_ms_divider = 0U;
        Key_Sample();
    }

    Key_AccumPressedTime();
}

uint8_t Key_Read(uint8_t i)
{
    switch (i) {
        case 1U: return s_k1_down;
        case 2U: return s_k2_down;
        case 3U: return s_k3_down;
        case 4U: return s_k4_down;
        default: return 0U;
    }
}

uint8_t Key_ReadLongPress(uint8_t i)
{
    uint8_t ret = 0U;

    switch (i) {
        case 1U:
            ret = s_k1_long_event;
            s_k1_long_event = 0U;
            return ret;
        case 2U:
            ret = s_k2_long_event;
            s_k2_long_event = 0U;
            return ret;
        case 3U:
            ret = s_k3_long_event;
            s_k3_long_event = 0U;
            return ret;
        case 4U:
            ret = s_k4_long_event;
            s_k4_long_event = 0U;
            return ret;
        default:
            return 0U;
    }
}

uint16_t Key_GetPressedTimeMs(uint8_t i)
{
    switch (i) {
        case 1U: return s_k1_pressed_ms;
        case 2U: return s_k2_pressed_ms;
        case 3U: return s_k3_pressed_ms;
        case 4U: return s_k4_pressed_ms;
        default: return 0U;
    }
}