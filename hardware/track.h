/*
 * track.h — 循迹 PID 模块
 *
 * 8 路红外循迹传感器，数据格式：
 *   bit7=最左侧  ...  bit0=最右侧（检测到黑线=1）
 *
 * 偏差计算：质心法 → 线位置(0~7) → 减中心(3.5) → 偏移量
 * PID 输出：差速修正值，叠加到左右轮基速上
 */

#ifndef __TRACK_H
#define __TRACK_H

#include <stdint.h>

void  Track_Init(void);

/* 输入 8bit 传感器数据，返回质心偏差（单位：传感器间距） */
float Track_CalcOffset(uint8_t sensorByte);

/*
 * PID 计算：输入偏差，输出差速修正值（单位：脉冲/20ms）
 * dt_s 为两次调用间的时间间隔（秒）
 */
float Track_PidUpdate(float centerOffset, float dt_s);

/* ---- 在线调参接口（供菜单使用） ---- */
float Track_GetKp(void);
void  Track_SetKp(float v);
float Track_GetKi(void);
void  Track_SetKi(float v);
float Track_GetKd(void);
void  Track_SetKd(float v);
float Track_GetOutMax(void);
void  Track_SetOutMax(float v);
float Track_GetIMax(void);
void  Track_SetIMax(float v);

/* 循迹基速（脉冲/20ms），菜单在 SPEED_PID 页编辑 */
int16_t Track_GetBaseSpeed(void);
void    Track_SetBaseSpeed(int16_t v);

#endif /* __TRACK_H */
