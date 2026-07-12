/*
 *  motor.h  —— 四路直流电机驱动（TIMG PWM + GPIO 方向控制）
 *
 *  支持：
 *    - 开环占空比控制
 *    - 有符号速度控制（适配 PID 闭环）
 *    - 死区保护，避免方向引脚高频翻转
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ---- 方向枚举 ---- */
typedef enum {
    MOTOR_DIR_STOP     = 0,
    MOTOR_DIR_FORWARD  = 1,
    MOTOR_DIR_BACKWARD = 2
} Motor_Dir;

/* ---- 单路电机配置 ---- */
typedef struct {
    /* 方向引脚 */
    GPIO_Regs   *dir1Port;
    uint32_t     dir1Pin;
    GPIO_Regs   *dir2Port;
    uint32_t     dir2Pin;

    /* PWM */
    GPTIMER_Regs *pwmInst;
    uint32_t      pwmCh;        /* DL_TIMER_CC_0_INDEX 或 DL_TIMER_CC_1_INDEX */
    uint32_t      pwmPeriod;    /* PWM 周期（对应 syscfg timerCount） */

    /* 死区 */
    int32_t       deadband;     /* 速度死区阈值（≥0） */
} Motor_Cfg;

/* ---- API ---- */

/*
 * 设置电机方向（内部函数，也可直接调用）
 *  dir: MOTOR_DIR_FORWARD  → dir1=H, dir2=L
 *       MOTOR_DIR_BACKWARD → dir1=L, dir2=H
 *       MOTOR_DIR_STOP     → dir1=L, dir2=L
 */
void Motor_SetDir(const Motor_Cfg *cfg, Motor_Dir dir);

/*
 * 设置 PWM 原始占空比（0 ~ pwmPeriod）
 */
void Motor_SetDuty(const Motor_Cfg *cfg, uint32_t duty);

/*
 * 有符号速度控制（闭环核心接口）
 *  speed > 0  → 正转，占空比 =  speed
 *  speed = 0  → 停止（考虑死区）
 *  speed < 0  → 反转，占空比 = -speed
 *
 *  自动处理：
 *    - 死区过滤
 *    - 占空比饱和限幅
 *    - 方向切换
 */
void Motor_SetSpeed(const Motor_Cfg *cfg, int32_t speed);

/*
 * 停止电机（PWM=0，方向脚拉低）
 */
void Motor_Stop(const Motor_Cfg *cfg);

/* ---- 四路电机配置表 ---- */
extern Motor_Cfg gMotor[4];

/* 索引宏 */
#define MOTOR1  (&gMotor[0])
#define MOTOR2  (&gMotor[1])
#define MOTOR3  (&gMotor[2])
#define MOTOR4  (&gMotor[3])

#endif /* __MOTOR_H */
