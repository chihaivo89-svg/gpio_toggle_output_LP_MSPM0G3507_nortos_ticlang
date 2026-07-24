/*
 * menu.h — 菜单系统（OLED 128x64, 7行 × 16字符）
 *
 * 按键语义：
 *   KEY1 = 确认/进入    KEY2 = 上移
 *   KEY3 = 返回上级     KEY4 = 下移
 *
 * 页面：
 *   MAIN → CONFIG / DATA_SHOW / MODE / START / RESET
 *   CONFIG → MODE1~4 列表 → MODE_CONFIG(某mode的参数)
 *   MODE_CONFIG → PID组选择 → PID_EDIT
 *   MODE → MODE1~4 选择
 *   DATA_SHOW → 实时数据只读显示
 */

#ifndef __MENU_H
#define __MENU_H

#include <stdint.h>

/* ---- 页面枚举 ---- */
typedef enum {
    PAGE_MAIN,
    PAGE_CONFIG,            /* 选择 MODE1~4 进入其参数配置 */
    PAGE_MODE,
    PAGE_DATA_SHOW,
    PAGE_MODE_CONFIG,       /* 某 mode 的 PID 组选择 */
    PAGE_PID_EDIT,          /* PID 参数编辑页（选/编 状态机） */
    PAGE_NR02_DBG,          /* 2.4G 无线调试 */
} MenuPage;

/* ---- PID 组（用于 MODE_CONFIG 页） ---- */
typedef enum {
    PID_GROUP_TRACK,        /* 循迹 PID */
    PID_GROUP_SPEED,        /* 速度环 PID */
    PID_GROUP_COUNT,
} PidGroup;

/* ---- PID 编辑字段 ---- */
typedef enum {
    PID_FIELD_KP = 1,
    PID_FIELD_KI,
    PID_FIELD_KD,
    PID_FIELD_OUT_MX,
    PID_FIELD_I_MX,    PID_FIELD_TURN_SPD,       /* 循迹转弯速度 */    PID_FIELD_BASE_SPD,       /* 仅 SPEED PID 生效 */
    PID_FIELD_COUNT = PID_FIELD_BASE_SPD,
} PidField;

/* ---- PID 编辑状态 ---- */
typedef enum {
    PID_STATE_SEL  = 0,    /* 选择字段（'>'光标） */
    PID_STATE_EDIT = 1,    /* 编辑字段（'-'光标） */
} PidEditState;

/* ---- 全局标志位（供 ttop_funtion.c 读取） ---- */
extern volatile uint8_t g_menuMode;        /* 当前模式 1~4 */
extern volatile uint8_t g_menuStartFlag;   /* 启动标志 */
extern volatile uint8_t g_menuResetFlag;   /* 复位标志 */

/* ---- NR02 调试（供 menu.c 读取） ---- */
extern volatile uint8_t g_nr02RxData;      /* 最近收到的无线数据 */

/* ---- 菜单公开 API ---- */
void Menu_Init(void);
void Menu_HandleKeys(void);
void Menu_Draw(void);

#endif /* __MENU_H */
