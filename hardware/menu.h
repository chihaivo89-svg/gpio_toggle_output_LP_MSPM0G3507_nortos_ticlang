/*
 * menu.h — 菜单系统（OLED 128x64, 7行 × 16字符）
 *
 * 按键语义：
 *   KEY1 = 确认/进入    KEY2 = 上移
 *   KEY3 = 返回上级     KEY4 = 下移
 *
 * 页面：
 *   MAIN → CONFIG / DATA_SHOW / MODE / START / RESET
 *   CONFIG → PID目标选择 / PID_CHANGE(预留)
 *   MODE → MODE1~4 选择
 *   DATA_SHOW → 实时数据只读显示
 */

#ifndef __MENU_H
#define __MENU_H

#include <stdint.h>

/* ---- 页面枚举 ---- */
typedef enum {
    PAGE_MAIN,
    PAGE_CONFIG,
    PAGE_MODE,
    PAGE_DATA_SHOW,
} MenuPage;

/* ---- PID 调整对象 ---- */
typedef enum {
    PID_TARGET_TRACE,   /* 循迹 PID */
    PID_TARGET_SPEED,   /* 速度环 PID */
    PID_TARGET_YAW,     /* 航向环 PID */
    PID_TARGET_COUNT,
} PidTarget;

/* ---- 全局标志位（供 ttop_funtion.c 读取） ---- */
extern volatile uint8_t g_menuMode;        /* 当前模式 1~4 */
extern volatile uint8_t g_menuStartFlag;   /* 启动标志 */
extern volatile uint8_t g_menuResetFlag;   /* 复位标志 */

/* ---- 菜单公开 API ---- */
void Menu_Init(void);
void Menu_HandleKeys(void);
void Menu_Draw(void);

#endif /* __MENU_H */
