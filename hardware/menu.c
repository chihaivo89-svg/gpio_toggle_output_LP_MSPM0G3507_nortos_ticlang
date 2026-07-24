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
#include "track.h"
#include "param_storage.h"
#include "top_zhengfangxing.h"
#include "si24r1.h"
#include <stdio.h>

/* ---- 全局标志位定义 ---- */
volatile uint8_t g_menuMode       = 1U;
volatile uint8_t g_menuStartFlag  = 0U;
volatile uint8_t g_menuResetFlag  = 0U;

/* ---- 循迹串口字节（引用自主程序） ---- */
extern volatile uint8_t gRxByte;

/* ---- NR02 接收数据 ---- */
extern volatile uint8_t g_nr02RxData;

/* ---- 内部状态 ---- */
static volatile MenuPage     s_page       = PAGE_MAIN;
static volatile uint8_t      s_cursor     = 1U;
static volatile uint8_t      s_dataPage   = 0U;       /* DATA_SHOW 子页 0/1 */
static volatile PidGroup     s_pidGroup   = PID_GROUP_SPEED;
static volatile PidField     s_pidField   = PID_FIELD_KP;
static volatile PidEditState s_pidState   = PID_STATE_SEL;

/* 保存成功提示帧计时（>0 时在 row 0 末尾显示 SAVE_OK） */
static volatile uint8_t s_saveOkFrames = 0U;

/* NR02 调试：待发送数据字节 */
static volatile uint8_t s_nr02TxData = 0U;


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

static const char *PidGroup_Name(PidGroup g)
{
    switch (g) {
        case PID_GROUP_TRACK: return "TRACK";
        case PID_GROUP_SPEED: return "SPEED";
        default:              return "?    ";
    }
}

static const char *PidField_Name(PidField f)
{
    switch (f) {
        case PID_FIELD_KP:       return "KP    ";
        case PID_FIELD_KI:       return "KI    ";
        case PID_FIELD_KD:       return "KD    ";
        case PID_FIELD_OUT_MX:   return "OUT_MX";
        case PID_FIELD_I_MX:     return "I_MX  ";
        case PID_FIELD_TURN_SPD: return "TURN_S";
        case PID_FIELD_BASE_SPD: return "BASE_S";
        default:                 return "?     ";
    }
}

/* 读当前 mode 的 PID 字段值（从 g_modeParams 直接读取） */
static float Pid_GetValue(PidGroup g, PidField f)
{
    const ModeParams *p = &g_modeParams[g_menuMode - 1U];

    if (f == PID_FIELD_BASE_SPD || f == PID_FIELD_TURN_SPD) {
        return (f == PID_FIELD_TURN_SPD) ? (float)p->trackTurnSpeed
                                         : (float)p->trackBaseSpeed;
    }
    if (g == PID_GROUP_TRACK) {
        switch (f) {
            case PID_FIELD_KP:     return p->trackKp;
            case PID_FIELD_KI:     return p->trackKi;
            case PID_FIELD_KD:     return p->trackKd;
            case PID_FIELD_OUT_MX: return p->trackOutMax;
            case PID_FIELD_I_MX:   return p->trackIMax;
        }
    } else {
        switch (f) {
            case PID_FIELD_KP:     return p->speedKp;
            case PID_FIELD_KI:     return p->speedKi;
            case PID_FIELD_KD:     return p->speedKd;
            case PID_FIELD_OUT_MX: return (float)p->speedOutLimit;
            case PID_FIELD_I_MX:   return (float)p->speedOutLimit;
        }
    }
    return 0.0f;
}

/* 写当前 mode 的 PID 字段值（直接写入 g_modeParams） */
static void Pid_SetValue(PidGroup g, PidField f, float v)
{
    ModeParams *p = &g_modeParams[g_menuMode - 1U];

    if (f == PID_FIELD_TURN_SPD) {
        if (v > 0.0f && v <= 50.0f) p->trackTurnSpeed = (int16_t)v;
        return;
    }
    if (f == PID_FIELD_BASE_SPD) {
        if (v > 0.0f && v <= 200.0f) p->trackBaseSpeed = (int16_t)v;
        return;
    }
    if (g == PID_GROUP_TRACK) {
        switch (f) {
            case PID_FIELD_KP:     if (v >= 0.0f) p->trackKp = v; break;
            case PID_FIELD_KI:     if (v >= 0.0f) p->trackKi = v; break;
            case PID_FIELD_KD:     if (v >= 0.0f) p->trackKd = v; break;
            case PID_FIELD_OUT_MX: if (v > 0.0f) { p->trackOutMax = v; } break;
            case PID_FIELD_I_MX:   if (v > 0.0f) { p->trackIMax = v; } break;
        }
    } else {
        switch (f) {
            case PID_FIELD_KP:     if (v >= 0.0f) p->speedKp = v; break;
            case PID_FIELD_KI:     if (v >= 0.0f) p->speedKi = v; break;
            case PID_FIELD_KD:     if (v >= 0.0f) p->speedKd = v; break;
            case PID_FIELD_OUT_MX: if (v > 0.0f && v <= 1000.0f) p->speedOutLimit = (int16_t)v; break;
            case PID_FIELD_I_MX:   if (v > 0.0f && v <= 1000.0f) p->speedOutLimit = (int16_t)v; break;
        }
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
    OLED_ShowString(0, 0, (uint8_t *)"MAIN", OLED_FONT_H);

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

    OLED_ShowString(0, 7, (uint8_t *)"MADE_BY->hhttiuui", OLED_FONT_H);
}

static void DrawConfig(void)
{
    uint8_t i;
    const char *items[5] = {"MODE1", "MODE2", "MODE3", "MODE4", "NR02_DBG"};

    OLED_ShowString(0, 0, (uint8_t *)"CONFIG", OLED_FONT_H);

    for (i = 1U; i <= 5U; i++) {
        sprintf(s_buf, "%c%s", (s_cursor == i) ? '>' : ' ', items[i - 1U]);
        OLED_ShowString(0, i, (uint8_t *)s_buf, OLED_FONT_H);
    }

    OLED_ShowString(0, 7, (uint8_t *)"MADE_BY->hhttiuui", OLED_FONT_H);
}

static void DrawModeConfig(void)
{
    uint8_t i;
    uint8_t groupCount = 0U;

    /* MODE2：TRACK_PID / SPEED_PID / TURN_S / ENTRY / E_THR / TURNS */
    if (g_menuMode == 2U) {
        groupCount = 6U;
    }

    sprintf(s_buf, "M%d_CFG", (int)g_menuMode);
    OLED_ShowString(0, 0, (uint8_t *)s_buf, OLED_FONT_H);

    if (groupCount == 0U) {
        OLED_ShowString(0, 2, (uint8_t *)" (no config) ", OLED_FONT_H);
    } else {
        for (i = 0U; i < groupCount; i++) {
            uint8_t row = i + 1U;
            ModeParams *p = &g_modeParams[g_menuMode - 1U];

            if (i == 0U) {
                /* TRACK_PID */
                sprintf(s_buf, "%cTRACK_PID", (s_cursor == row) ? '>' : ' ');
            } else if (i == 1U) {
                /* SPEED_PID */
                sprintf(s_buf, "%cSPEED_PID", (s_cursor == row) ? '>' : ' ');
            } else if (i == 2U) {
                /* TURN_S:xx 可直接编辑 */
                uint8_t editing = (s_cursor == row && s_pidState == PID_STATE_EDIT) ? 1U : 0U;
                float spd = (float)p->trackTurnSpeed;
                char cur = editing ? '-' : ((s_cursor == row) ? '>' : ' ');
                sprintf(s_buf, "%cTURN_S:%7.2f", cur, (double)spd);
            } else if (i == 3U) {
                /* ENTRY:ON/OFF 开关 */
                char cur = (s_cursor == row) ? '>' : ' ';
                sprintf(s_buf, "%cENTRY: %s", cur, p->entryEnable ? "ON " : "OFF");
            } else if (i == 4U) {
                /* E_THR:xxxx 可直接编辑 */
                uint8_t editing = (s_cursor == row && s_pidState == PID_STATE_EDIT) ? 1U : 0U;
                char cur = editing ? '-' : ((s_cursor == row) ? '>' : ' ');
                sprintf(s_buf, "%cE_THR: %5d", cur, (int)p->entryThreshold);
            } else if (i == 5U) {
                /* TURNS: x/xx 显示当前计数/目标 */
                uint8_t editing = (s_cursor == row && s_pidState == PID_STATE_EDIT) ? 1U : 0U;
                char cur = editing ? '-' : ((s_cursor == row) ? '>' : ' ');
                if (p->stopAfterTurns > 0) {
                    sprintf(s_buf, "%cTURNS:%3d/%3d", cur,
                            (int)TrackSquare_GetTurnCount(), (int)p->stopAfterTurns);
                } else {
                    sprintf(s_buf, "%cTURNS:%3d/ --", cur,
                            (int)TrackSquare_GetTurnCount());
                }
            }
            OLED_ShowString(0, row, (uint8_t *)s_buf, OLED_FONT_H);
        }
    }

    OLED_ShowString(0, 7, (uint8_t *)"MADE_BY->hhttiuui", OLED_FONT_H);
}

static void DrawMode(void)
{
    uint8_t i;

    OLED_ShowString(0, 0, (uint8_t *)"MODE", OLED_FONT_H);

    for (i = 1U; i <= 4U; i++) {
        sprintf(s_buf, "%c%s", (s_cursor == i) ? '>' : ' ', Mode_Name(i));
        OLED_ShowString(0, i, (uint8_t *)s_buf, OLED_FONT_H);
    }

    OLED_ShowString(0, 7, (uint8_t *)"MADE_BY->hhttiuui", OLED_FONT_H);
}

/* trace 字节解码为二进制字符串 "0000 0001" */
static void DecodeTraceBits(uint8_t byte, char *out)
{
    uint8_t j;
    for (j = 0U; j < 8U; j++) {
        if (j == 4U) { *out++ = ' '; }
        *out++ = (byte & (0x01U << j)) ? '1' : '0';
    }
    *out = '\0';
}

static void DrawDataShow(void)
{
    OLED_ShowString(0, 0, (uint8_t *)"DATASHOW", OLED_FONT_H);

    if (s_dataPage == 0U) {
        /* ---- Page 0: TRACE + 编码器 + 里程 ---- */
        char bits[12];

        /* Row 1: TRACE */
        sprintf(s_buf, "TRACE:      %02X", gRxByte);
        OLED_ShowString(0, 1, (uint8_t *)s_buf, OLED_FONT_H);

        /* Row 2: 二进制解码 */
        DecodeTraceBits(gRxByte, bits);
        sprintf(s_buf, "      %s", bits);
        OLED_ShowString(0, 2, (uint8_t *)s_buf, OLED_FONT_H);

        /* Row 3: SPD1 */
        sprintf(s_buf, "SPD1:   %5d", (int)Encoder_GetPulses(&gEncMotor1));
        OLED_ShowString(0, 3, (uint8_t *)s_buf, OLED_FONT_H);

        /* Row 4: SPD2 */
        sprintf(s_buf, "SPD2:   %5d", (int)Encoder_GetPulses(&gEncMotor3));
        OLED_ShowString(0, 4, (uint8_t *)s_buf, OLED_FONT_H);

        /* Row 5: M1 累计里程 */
        sprintf(s_buf, "MILE1:  %5d", (int)Encoder_GetTotalPulses(&gEncMotor1));
        OLED_ShowString(0, 5, (uint8_t *)s_buf, OLED_FONT_H);

        /* Row 6: M3 累计里程 */
        sprintf(s_buf, "MILE3:  %5d", (int)Encoder_GetTotalPulses(&gEncMotor3));
        OLED_ShowString(0, 6, (uint8_t *)s_buf, OLED_FONT_H);
    } else {
        /* ---- Page 1: 姿态 ---- */
        /* Row 1: YAW */
        sprintf(s_buf, "YAW:     %6.1f", (double)euler.angle.yaw);
        OLED_ShowString(0, 1, (uint8_t *)s_buf, OLED_FONT_H);

        /* Row 2: ROLL */
        sprintf(s_buf, "ROLL:    %6.1f", (double)euler.angle.roll);
        OLED_ShowString(0, 2, (uint8_t *)s_buf, OLED_FONT_H);

        /* Row 3: PITCH */
        sprintf(s_buf, "PITCH:   %6.1f", (double)euler.angle.pitch);
        OLED_ShowString(0, 3, (uint8_t *)s_buf, OLED_FONT_H);
    }

    /* Row 7: 提示 + 页码 */
    sprintf(s_buf, "MADE_BY->hhttiuui P%d", (int)s_dataPage);
    OLED_ShowString(0, 7, (uint8_t *)s_buf, OLED_FONT_H);
}

static void DrawNR02Dbg(void)
{
    char cursorCh = (s_pidState == PID_STATE_EDIT) ? '-' : '>';

    OLED_ShowString(0, 0, (uint8_t *)"NR02_DBG", OLED_FONT_H);

    /* Row 1: T_DATA:xx（支持编辑态光标切换） */
    sprintf(s_buf, "%cT_DATA:%3d",
            (s_cursor == 1U) ? cursorCh : ' ', (int)s_nr02TxData);
    OLED_ShowString(0, 1, (uint8_t *)s_buf, OLED_FONT_H);

    /* Row 2: R_DATA:xx（只读） */
    sprintf(s_buf, "  R_DATA:%3d", (int)g_nr02RxData);
    OLED_ShowString(0, 2, (uint8_t *)s_buf, OLED_FONT_H);

    /* Row 3: TRANSMIT */
    sprintf(s_buf, "%cTRANSMIT", (s_cursor == 3U) ? '>' : ' ');
    OLED_ShowString(0, 3, (uint8_t *)s_buf, OLED_FONT_H);

    OLED_ShowString(0, 7, (uint8_t *)"MADE_BY->hhttiuui", OLED_FONT_H);
}

static void DrawPidEdit(void)
{
    uint8_t i, row;
    const char *title = PidGroup_Name(s_pidGroup);
    char cursorCh = (s_pidState == PID_STATE_EDIT) ? '-' : '>';
    /* TRACK 页: KP..I_MX (5字段); SPEED 页: KP..BASE_SPD (跳过I_MX和TURN_S) */
    uint8_t maxField = (s_pidGroup == PID_GROUP_TRACK)
                       ? (uint8_t)PID_FIELD_I_MX
                       : (uint8_t)PID_FIELD_COUNT;

    /* Row 0: 标题 */
    sprintf(s_buf, "%s_PID", title);
    OLED_ShowString(0, 0, (uint8_t *)s_buf, OLED_FONT_H);

    row = 1U;
    for (i = 1U; i <= maxField; i++) {
        /* SPEED 页跳过 I_MX 和 TURN_S */
        if (s_pidGroup != PID_GROUP_TRACK &&
            ((PidField)i == PID_FIELD_I_MX || (PidField)i == PID_FIELD_TURN_SPD)) {
            continue;
        }
        const char *name = PidField_Name((PidField)i);
        float val = Pid_GetValue(s_pidGroup, (PidField)i);

        if (i == (uint8_t)s_pidField) {
            sprintf(s_buf, "%c%s:%7.2f", cursorCh, name, (double)val);
        } else {
            sprintf(s_buf, " %s:%7.2f", name, (double)val);
        }
        OLED_ShowString(0, row, (uint8_t *)s_buf, OLED_FONT_H);
        row++;
    }

    /* Row 7: 提示 */
    OLED_ShowString(0, 7, (uint8_t *)"MADE_BY->hhttiuui", OLED_FONT_H);
}

/* ---- 确认处理 ---- */

static void Main_OnConfirm(void)
{
    switch (s_cursor) {
        case 1U: s_page = PAGE_CONFIG;     s_cursor = 1U; break;
        case 2U: s_page = PAGE_DATA_SHOW;  s_cursor = 1U; s_dataPage = 0U; break;
        case 3U: s_page = PAGE_MODE;       s_cursor = 1U; break;
        case 4U: g_menuStartFlag = 1U; g_menuResetFlag = 0U; break;
        case 5U: g_menuResetFlag = 1U; g_menuStartFlag = 0U; break;
        default: break;
    }
}

static void Config_OnConfirm(void)
{
    if (s_cursor <= 4U) {
        g_menuMode = s_cursor;
        s_page     = PAGE_MODE_CONFIG;
        s_cursor   = 1U;
        s_pidState = PID_STATE_SEL;
    } else {
        s_page   = PAGE_NR02_DBG;
        s_cursor = 1U;
        s_pidState = PID_STATE_SEL;
    }
}

static void ModeConfig_OnConfirm(void)
{
    s_pidGroup  = (s_cursor == 1U) ? PID_GROUP_TRACK : PID_GROUP_SPEED;
    s_pidField  = PID_FIELD_KP;
    s_page      = PAGE_PID_EDIT;
    s_pidState  = PID_STATE_SEL;
}

static void Mode_OnConfirm(void)
{
    g_menuMode = s_cursor;
    s_page     = PAGE_MAIN;
    s_cursor   = 3U;
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
    static uint8_t s_comboLatched = 0U;

    uint8_t key = 0U;
    if (k1_now && !k1_last) { key = KEY1_PRES; }
    if (k2_now && !k2_last) { key = KEY2_PRES; }
    if (k3_now && !k3_last) { key = KEY3_PRES; }
    if (k4_now && !k4_last) { key = KEY4_PRES; }

    k1_last = k1_now;
    k2_last = k2_now;
    k3_last = k3_now;
    k4_last = k4_now;

    /* ===== 组合键：KEY2(上) + KEY4(下) 同时按下 → 保存参数 ===== */
    if (k2_now && k4_now) {
        if (!s_comboLatched) {
            s_comboLatched = 1U;
            Param_SaveAll();
            s_saveOkFrames = 15U;  /* 保持约 15 帧（~250ms） */
        }
        k1_last = k1_now;  k2_last = k2_now;
        k3_last = k3_now;  k4_last = k4_now;
        return;
    } else {
        s_comboLatched = 0U;
    }

    if (key == 0U) {
        return;
    }

    /* ===== 全局 KEY1=返回：链式回到对应上级 ===== */
    if (key == KEY1_PRES && s_page != PAGE_MAIN) {
        if (s_page == PAGE_PID_EDIT) {
            s_page = PAGE_MODE_CONFIG;  s_cursor = 1U;
        } else if (s_page == PAGE_MODE_CONFIG) {
            s_page = PAGE_CONFIG;       s_cursor = g_menuMode;
        } else if (s_page == PAGE_NR02_DBG) {
            s_page = PAGE_CONFIG;       s_cursor = 5U;
        } else {
            s_page = PAGE_MAIN;         s_cursor = 1U;
        }
        OLED_Clear();
        return;
    }

    /* ===== 按页面分发 ===== */
    switch (s_page) {

        case PAGE_MAIN:
            if (key == KEY2_PRES) {
                if (s_cursor > 1U) s_cursor--; else s_cursor = 5U;
                OLED_Clear();
            } else if (key == KEY4_PRES) {
                if (s_cursor < 5U) s_cursor++; else s_cursor = 1U;
                OLED_Clear();
            } else if (key == KEY3_PRES) {
                Main_OnConfirm();
                OLED_Clear();
            }
            break;

        case PAGE_CONFIG:
            if (key == KEY2_PRES) {
                if (s_cursor > 1U) s_cursor--; else s_cursor = 5U;
                OLED_Clear();
            } else if (key == KEY4_PRES) {
                if (s_cursor < 5U) s_cursor++; else s_cursor = 1U;
                OLED_Clear();
            } else if (key == KEY3_PRES) {
                Config_OnConfirm();
                OLED_Clear();
            }
            break;

        case PAGE_MODE_CONFIG:
            {
                ModeParams *p = &g_modeParams[g_menuMode - 1U];

                /* TURN_S 编辑态（光标在第3行） */
                if (s_cursor == 3U && s_pidState == PID_STATE_EDIT) {
                    if (key == KEY2_PRES) {
                        p->trackTurnSpeed += 1;
                        if (p->trackTurnSpeed > 50) p->trackTurnSpeed = 50;
                        OLED_Clear();
                    } else if (key == KEY4_PRES) {
                        p->trackTurnSpeed -= 1;
                        if (p->trackTurnSpeed < 0) p->trackTurnSpeed = 0;
                        OLED_Clear();
                    } else if (key == KEY3_PRES) {
                        s_pidState = PID_STATE_SEL;
                        OLED_Clear();
                    } else if (key == KEY1_PRES) {
                        s_pidState = PID_STATE_SEL;
                        OLED_Clear();
                    }
                    break;
                }

                /* E_THR 编辑态（光标在第5行） */
                if (s_cursor == 5U && s_pidState == PID_STATE_EDIT) {
                    if (key == KEY2_PRES) {
                        p->entryThreshold += 10;
                        if (p->entryThreshold > 2000) p->entryThreshold = 2000;
                        OLED_Clear();
                    } else if (key == KEY4_PRES) {
                        p->entryThreshold -= 10;
                        if (p->entryThreshold < 10) p->entryThreshold = 10;
                        OLED_Clear();
                    } else if (key == KEY3_PRES) {
                        s_pidState = PID_STATE_SEL;
                        OLED_Clear();
                    } else if (key == KEY1_PRES) {
                        s_pidState = PID_STATE_SEL;
                        OLED_Clear();
                    }
                    break;
                }

                /* TURNS 编辑态（光标在第6行） */
                if (s_cursor == 6U && s_pidState == PID_STATE_EDIT) {
                    if (key == KEY2_PRES) {
                        p->stopAfterTurns++;
                        if (p->stopAfterTurns > 99) p->stopAfterTurns = 99;
                        OLED_Clear();
                    } else if (key == KEY4_PRES) {
                        p->stopAfterTurns--;
                        if (p->stopAfterTurns < 0) p->stopAfterTurns = 0;
                        OLED_Clear();
                    } else if (key == KEY3_PRES) {
                        s_pidState = PID_STATE_SEL;
                        OLED_Clear();
                    } else if (key == KEY1_PRES) {
                        s_pidState = PID_STATE_SEL;
                        OLED_Clear();
                    }
                    break;
                }

                /* ENTRY 开关切换（光标在第4行） */
                if (s_cursor == 4U && key == KEY3_PRES) {
                    p->entryEnable = p->entryEnable ? 0 : 1;
                    OLED_Clear();
                    break;
                }

                /* 光标移动 */
                if (key == KEY2_PRES) {
                    if (s_cursor > 1U) s_cursor--; else s_cursor = 6U;
                    OLED_Clear();
                } else if (key == KEY4_PRES) {
                    if (s_cursor < 6U) s_cursor++; else s_cursor = 1U;
                    OLED_Clear();
                } else if (key == KEY3_PRES) {
                    /* 进入编辑态（TURN_S / E_THR / TURNS）或进入 PID 页面 */
                    if (s_cursor == 3U || s_cursor == 5U || s_cursor == 6U) {
                        s_pidState = PID_STATE_EDIT;
                        OLED_Clear();
                    } else {
                        ModeConfig_OnConfirm();
                        OLED_Clear();
                    }
                }
            }
            break;

        case PAGE_MODE:
            if (key == KEY2_PRES) {
                if (s_cursor > 1U) s_cursor--; else s_cursor = 4U;
                OLED_Clear();
            } else if (key == KEY4_PRES) {
                if (s_cursor < 4U) s_cursor++; else s_cursor = 1U;
                OLED_Clear();
            } else if (key == KEY3_PRES) {
                Mode_OnConfirm();
                OLED_Clear();
            }
            break;

        case PAGE_DATA_SHOW:
            if (key == KEY2_PRES || key == KEY4_PRES) {
                s_dataPage ^= 1U;
                OLED_Clear();
            }
            break;

        case PAGE_NR02_DBG:
            /* T_DATA 编辑态 */
            if (s_cursor == 1U && s_pidState == PID_STATE_EDIT) {
                if (key == KEY2_PRES) {
                    s_nr02TxData++;
                    OLED_Clear();
                } else if (key == KEY4_PRES) {
                    s_nr02TxData--;
                    OLED_Clear();
                } else if (key == KEY3_PRES || key == KEY1_PRES) {
                    s_pidState = PID_STATE_SEL;
                    OLED_Clear();
                }
            } else if (s_cursor == 1U && key == KEY3_PRES) {
                s_pidState = PID_STATE_EDIT;
                OLED_Clear();
            } else if (s_cursor == 3U && key == KEY3_PRES) {
                /* 发送当前数据 */
                uint8_t txBuf[32] = {0};
                txBuf[0] = s_nr02TxData;
                Si24R1_TxPacket(txBuf);
                OLED_Clear();
            } else if (key == KEY2_PRES) {
                if (s_cursor > 1U) s_cursor--; else s_cursor = 3U;
                OLED_Clear();
            } else if (key == KEY4_PRES) {
                if (s_cursor < 3U) s_cursor++; else s_cursor = 1U;
                OLED_Clear();
            }
            break;

        case PAGE_PID_EDIT:
            if (s_pidState == PID_STATE_SEL) {
                if (key == KEY2_PRES) {
                    do {
                        if (s_pidField > (PidField)1U)
                            s_pidField = (PidField)((uint8_t)s_pidField - 1U);
                        else s_pidField = (PidField)PID_FIELD_COUNT;
                    } while (s_pidGroup != PID_GROUP_TRACK &&
                             ((PidField)s_pidField == PID_FIELD_I_MX ||
                              (PidField)s_pidField == PID_FIELD_TURN_SPD));
                    OLED_Clear();
                } else if (key == KEY4_PRES) {
                    do {
                        if (s_pidField < (PidField)PID_FIELD_COUNT)
                            s_pidField = (PidField)((uint8_t)s_pidField + 1U);
                        else s_pidField = PID_FIELD_KP;
                    } while (s_pidGroup != PID_GROUP_TRACK &&
                             ((PidField)s_pidField == PID_FIELD_I_MX ||
                              (PidField)s_pidField == PID_FIELD_TURN_SPD));
                    OLED_Clear();
                } else if (key == KEY3_PRES) {
                    s_pidState = PID_STATE_EDIT;
                    OLED_Clear();
                }
            } else {
                uint8_t k2_long = Key_ReadLongPress(KEY_ID_2);
                uint8_t k4_long = Key_ReadLongPress(KEY_ID_4);
                float val = Pid_GetValue(s_pidGroup, s_pidField);
                /* 整数字段（int16_t）步长 1.0，浮点字段步长 0.01 */
                uint8_t isIntField =
                    (PidField)s_pidField == PID_FIELD_BASE_SPD ||
                    (PidField)s_pidField == PID_FIELD_TURN_SPD ||
                    ((PidField)s_pidField == PID_FIELD_OUT_MX && s_pidGroup != PID_GROUP_TRACK) ||
                    ((PidField)s_pidField == PID_FIELD_I_MX   && s_pidGroup != PID_GROUP_TRACK);
                float step = (k2_long || k4_long) ? (isIntField ? 5.0f : 0.50f)
                                                  : (isIntField ? 1.0f : 0.01f);

                if (key == KEY2_PRES || (k2_long && k2_now)) {
                    Pid_SetValue(s_pidGroup, s_pidField, val + step);
                    OLED_Clear();
                } else if (key == KEY4_PRES || (k4_long && k4_now)) {
                    float next = val - step;
                    if (next < 0.0f) next = 0.0f;
                    Pid_SetValue(s_pidGroup, s_pidField, next);
                    OLED_Clear();
                } else if (key == KEY3_PRES) {
                    s_pidState = PID_STATE_SEL;
                    OLED_Clear();
                }
            }
            break;
    }
}

/*
 * 每帧在主循环中调用一次，根据当前页面覆写 OLED（不清屏）。
 * 清屏只在 Menu_HandleKeys() 中页面切换或光标移动时触发，避免闪烁。
 */
void Menu_Draw(void)
{
    switch (s_page) {
        case PAGE_MAIN:        DrawMain();       break;
        case PAGE_CONFIG:      DrawConfig();     break;
        case PAGE_MODE_CONFIG: DrawModeConfig(); break;
        case PAGE_MODE:        DrawMode();       break;
        case PAGE_DATA_SHOW:   DrawDataShow();   break;
        case PAGE_NR02_DBG:    DrawNR02Dbg();    break;
        case PAGE_PID_EDIT:    DrawPidEdit();    break;
    }

    /* 保存成功提示：在 row 0 末尾叠加 SAVE_OK */
    if (s_saveOkFrames > 0U) {
        s_saveOkFrames--;
        OLED_ShowString(8, 0, (uint8_t *)"SAVE_OK", OLED_FONT_H);
    }
}
