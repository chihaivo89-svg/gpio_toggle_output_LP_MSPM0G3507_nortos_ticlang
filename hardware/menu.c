/*
 * menu.c — 菜单系统实现
 *
 * 按键语义（与本工程按键顺序一致）：
 *   KEY1 = 返回上级    KEY2 = 上移光标
 *   KEY3 = 确认/进入   KEY4 = 下移光标
 *
 * 消抖边沿检测参考 TC264 Menu_Select() 模式。
 */

#include "menu.h"
#include "key.h"
#include "oled_hardware_i2c.h"
#include "encoder.h"
#include "IMU660RB/imu660rb.h"
#include "speed_control.h"

/* ---- 全局标志位定义 ---- */
volatile uint8_t g_menuMode       = 1U;
volatile uint8_t g_menuStartFlag  = 0U;
volatile uint8_t g_menuResetFlag  = 0U;

/* ---- 循迹串口字节（引用自主程序） ---- */
extern volatile uint8_t gRxByte;

/* ---- 内部状态 ---- */
static volatile MenuPage  s_page       = PAGE_MAIN;
static volatile uint8_t   s_cursor     = 1U;
static volatile PidTarget s_pidTarget  = PID_TARGET_SPEED;

/* OLED 8x8 字体，每行 16 字符，共 8 行（0~7） */
#define OLED_FONT_H   (8U)
#define OLED_ROWS     (8U)

/* 按键事件宏：对齐 TC264 Menu.c 的 KEYx_PRES 定义 */
#define KEY1_PRES  (1U)
#define KEY2_PRES  (2U)
#define KEY3_PRES  (3U)
#define KEY4_PRES  (4U)

/* ---- 辅助 ---- */

static char        s_buf[24];

static const char *PidTarget_Name(PidTarget t)
{
    switch (t) {
        case PID_TARGET_TRACE: return "TRACE";
        case PID_TARGET_SPEED: return "SPEED";
        case PID_TARGET_YAW:   return "YAW  ";
        default:               return "?    ";
    }
}

static const char *Mode_Name(uint8_t m)
{
    switch (m) {
        case 1U: return "MODE1";
        case 2U: return "MODE2";
        case 3U: return "MODE3";
        case 4U: return "MODE4";
        default: return "MODE?";
    }
}

/* ---- 页面绘制 ---- */

static void DrawMain(void)
{
    uint8_t i;
    const char *items[] = {
        "CONFIG", "DATA_SHOW", "MODE:X", "START", "RESET"
    };
    const uint8_t itemCount = 5U;

    /* Row 0: 标题 + 署名 */
    OLED_ShowString(0, 0, (uint8_t *)"MAIN         hhttiuui", OLED_FONT_H);

    /* Row 1~5: 菜单项 */
    for (i = 1U; i <= itemCount; i++) {
        const char *label = items[i - 1U];
        uint8_t prefix = (s_cursor == i) ? '>' : ' ';

        if (i == 3U) {
            /* MODE:X 行：显示当前模式号 */
            sprintf(s_buf, "%cMODE:%s", prefix, Mode_Name(g_menuMode));
            OLED_ShowString(0, i, (uint8_t *)s_buf, OLED_FONT_H);
        } else {
            sprintf(s_buf, "%c%s", prefix, label);
            OLED_ShowString(0, i, (uint8_t *)s_buf, OLED_FONT_H);
        }
    }

    /* Row 7: 按键提示 */
    OLED_ShowString(0, 7, (uint8_t *)"1< 2:^ 3:OK 4:v", OLED_FONT_H);
}

static void DrawConfig(void)
{
    const char *pidName = PidTarget_Name(s_pidTarget);

    OLED_ShowString(0, 0, (uint8_t *)"CONFIG       hhttiuui", OLED_FONT_H);

    /* Row 1: PID:XXX */
    sprintf(s_buf, "%cPID:%s", (s_cursor == 1U) ? '>' : ' ', pidName);
    OLED_ShowString(0, 1, (uint8_t *)s_buf, OLED_FONT_H);

    /* Row 2: PID_CHANGE */
    sprintf(s_buf, "%cPID_CHANGE", (s_cursor == 2U) ? '>' : ' ');
    OLED_ShowString(0, 2, (uint8_t *)s_buf, OLED_FONT_H);

    /* Row 7: 提示 */
    OLED_ShowString(0, 7, (uint8_t *)"1< 3:OK          ", OLED_FONT_H);
}

static void DrawMode(void)
{
    uint8_t i;

    OLED_ShowString(0, 0, (uint8_t *)"MODE         hhttiuui", OLED_FONT_H);

    for (i = 1U; i <= 4U; i++) {
        sprintf(s_buf, "%c%s", (s_cursor == i) ? '>' : ' ', Mode_Name(i));
        OLED_ShowString(0, i, (uint8_t *)s_buf, OLED_FONT_H);
    }

    OLED_ShowString(0, 7, (uint8_t *)"1< 3:SEL         ", OLED_FONT_H);
}

static void DrawDataShow(void)
{
    /* Row 0: 标题 + 署名 */
    OLED_ShowString(0, 0, (uint8_t *)"DATASHOW     hhttiuui", OLED_FONT_H);

    /* Row 1: TRACE */
    sprintf(s_buf, "TRACE:       %02X", gRxByte);
    OLED_ShowString(0, 1, (uint8_t *)s_buf, OLED_FONT_H);

    /* Row 2: YAW */
    sprintf(s_buf, "YAW:      %6.1f", euler.angle.yaw);
    OLED_ShowString(0, 2, (uint8_t *)s_buf, OLED_FONT_H);

    /* Row 3: ROLL */
    sprintf(s_buf, "ROLL:     %6.1f", euler.angle.roll);
    OLED_ShowString(0, 3, (uint8_t *)s_buf, OLED_FONT_H);

    /* Row 4: PITCH */
    sprintf(s_buf, "PITCH:    %6.1f", euler.angle.pitch);
    OLED_ShowString(0, 4, (uint8_t *)s_buf, OLED_FONT_H);

    /* Row 5: SPD1 */
    sprintf(s_buf, "SPD1:     %5d", (int)Encoder_GetPulses(&gEncMotor1));
    OLED_ShowString(0, 5, (uint8_t *)s_buf, OLED_FONT_H);

    /* Row 6: SPD2 */
    sprintf(s_buf, "SPD2:     %5d", (int)Encoder_GetPulses(&gEncMotor3));
    OLED_ShowString(0, 6, (uint8_t *)s_buf, OLED_FONT_H);

    /* Row 7: 提示 */
    OLED_ShowString(0, 7, (uint8_t *)"1:BACK          ", OLED_FONT_H);
}

/* ---- 按键分发 ---- */

/*
 * MAIN 页确认（KEY3）处理。
 * 光标位置：
 *   1=CONFIG   2=DATA_SHOW   3=MODE:X   4=START   5=RESET
 */
static void Main_OnConfirm(void)
{
    switch (s_cursor) {
        case 1U:
            s_page   = PAGE_CONFIG;
            s_cursor = 1U;
            break;
        case 2U:
            s_page   = PAGE_DATA_SHOW;
            s_cursor = 1U;
            break;
        case 3U:
            s_page   = PAGE_MODE;
            s_cursor = 1U;
            break;
        case 4U:
            g_menuStartFlag = 1U;
            g_menuResetFlag = 0U;
            break;
        case 5U:
            g_menuResetFlag = 1U;
            g_menuStartFlag = 0U;
            break;
        default:
            break;
    }
}

/*
 * CONFIG 页确认（KEY3）处理。
 *   光标=1 → 循环 PID 目标
 *   光标=2 → 进入 PID_CHANGE（预留）
 */
static void Config_OnConfirm(void)
{
    if (s_cursor == 1U) {
        /* 循环 PID 目标 */
        int next = (int)s_pidTarget + 1;
        if (next >= (int)PID_TARGET_COUNT) {
            next = 0;
        }
        s_pidTarget = (PidTarget)next;
    } else {
        /* PID_CHANGE: 预留，当前空操作 */
    }
}

/*
 * MODE 页确认（KEY3）处理：选中当前光标对应的模式，自动返回 MAIN。
 */
static void Mode_OnConfirm(void)
{
    g_menuMode  = s_cursor;    /* 1 → MODE1, 2 → MODE2 ... */
    s_page      = PAGE_MAIN;
    s_cursor    = 3U;          /* 回到 MAIN 的 MODE:X 行 */
}

/* ---- 公开 API ---- */

void Menu_Init(void)
{
    s_page   = PAGE_MAIN;
    s_cursor = 1U;
    OLED_Clear();
}

/*
 * 按键边沿检测，每帧在主循环中调用一次。
 * 参考 TC264 Menu_Select() 的 static 边沿变量模式。
 */
void Menu_HandleKeys(void)
{
    uint8_t k1_now = Key_Read(KEY_ID_1);
    uint8_t k2_now = Key_Read(KEY_ID_2);
    uint8_t k3_now = Key_Read(KEY_ID_3);
    uint8_t k4_now = Key_Read(KEY_ID_4);

    static uint8_t k1_last, k2_last, k3_last, k4_last;

    uint8_t key = 0U;
    if (k1_now && !k1_last) { key = KEY1_PRES; }
    if (k2_now && !k2_last) { key = KEY2_PRES; }
    if (k3_now && !k3_last) { key = KEY3_PRES; }
    if (k4_now && !k4_last) { key = KEY4_PRES; }

    k1_last = k1_now;
    k2_last = k2_now;
    k3_last = k3_now;
    k4_last = k4_now;

    if (key == 0U) {
        return;
    }

    /* ===== 全局 KEY1=返回：任何子页面返回 MAIN ===== */
    if (key == KEY1_PRES && s_page != PAGE_MAIN) {
        s_page   = PAGE_MAIN;
        s_cursor = 1U;
        OLED_Clear();
        return;
    }

    /* ===== 按页面分发 ===== */
    switch (s_page) {

        case PAGE_MAIN:
            if (key == KEY2_PRES) {
                /* 上移 */
                if (s_cursor > 1U) { s_cursor--; }
                else               { s_cursor = 5U; }
                OLED_Clear();
            } else if (key == KEY4_PRES) {
                /* 下移 */
                if (s_cursor < 5U) { s_cursor++; }
                else               { s_cursor = 1U; }
                OLED_Clear();
            } else if (key == KEY3_PRES) {
                Main_OnConfirm();
                OLED_Clear();
            }
            /* KEY1 on MAIN: 无操作（已是顶层） */
            break;

        case PAGE_CONFIG:
            if (key == KEY2_PRES) {
                if (s_cursor > 1U) { s_cursor--; }
                else               { s_cursor = 2U; }
                OLED_Clear();
            } else if (key == KEY4_PRES) {
                if (s_cursor < 2U) { s_cursor++; }
                else               { s_cursor = 1U; }
                OLED_Clear();
            } else if (key == KEY3_PRES) {
                Config_OnConfirm();
                OLED_Clear();
            }
            break;

        case PAGE_MODE:
            if (key == KEY2_PRES) {
                if (s_cursor > 1U) { s_cursor--; }
                else               { s_cursor = 4U; }
                OLED_Clear();
            } else if (key == KEY4_PRES) {
                if (s_cursor < 4U) { s_cursor++; }
                else               { s_cursor = 1U; }
                OLED_Clear();
            } else if (key == KEY3_PRES) {
                Mode_OnConfirm();
                OLED_Clear();
            }
            break;

        case PAGE_DATA_SHOW:
            /* 只读页，KEY1 已在上方全局处理返回 */
            break;
    }
}

/*
 * 每帧在主循环中调用一次，根据当前页面绘制 OLED。
 * OLED 只有 8 行（0~7），每行最多 16 个 8x8 字符。
 */
/*
 * 每帧在主循环中调用一次，根据当前页面覆写 OLED（不清屏）。
 * 清屏只在 Menu_HandleKeys() 中页面切换或光标移动时触发，避免闪烁。
 */
void Menu_Draw(void)
{
    switch (s_page) {
        case PAGE_MAIN:       DrawMain();      break;
        case PAGE_CONFIG:     DrawConfig();    break;
        case PAGE_MODE:       DrawMode();      break;
        case PAGE_DATA_SHOW:  DrawDataShow();  break;
    }
}
