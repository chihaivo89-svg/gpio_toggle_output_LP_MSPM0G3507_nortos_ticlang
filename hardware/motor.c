/*
 *  motor.c  —— 四路直流电机驱动实现
 *
 *  PWM:  TIMG8 (PWM_0) / TIMG6 (PWM_1)
 *  DIR:  GPIO 数字输出（AIN1/AIN2, BIN1/BIN2, CIN1/CIN2, DIN1/DIN2）
 *
 *  H 桥真值表（以 L298N / TB6612 为例）：
 *    dir1=H, dir2=L  → 正转
 *    dir1=L, dir2=H  → 反转
 *    dir1=L, dir2=L  → 刹车/停止
 */

#include "motor.h"

/* ---- 方向控制 ---- */

void Motor_SetDir(const Motor_Cfg *cfg, Motor_Dir dir)
{
    switch (dir) {
    case MOTOR_DIR_FORWARD:
        DL_GPIO_setPins(cfg->dir1Port, cfg->dir1Pin);
        DL_GPIO_clearPins(cfg->dir2Port, cfg->dir2Pin);
        break;

    case MOTOR_DIR_BACKWARD:
        DL_GPIO_clearPins(cfg->dir1Port, cfg->dir1Pin);
        DL_GPIO_setPins(cfg->dir2Port, cfg->dir2Pin);
        break;

    default: /* MOTOR_DIR_STOP */
        DL_GPIO_clearPins(cfg->dir1Port, cfg->dir1Pin);
        DL_GPIO_clearPins(cfg->dir2Port, cfg->dir2Pin);
        break;
    }
}

/* ---- PWM 占空比 ---- */

void Motor_SetDuty(const Motor_Cfg *cfg, uint32_t duty)
{
    if (duty > cfg->pwmPeriod) {
        duty = cfg->pwmPeriod;
    }
    DL_TimerG_setCaptureCompareValue(cfg->pwmInst, duty, cfg->pwmCh);
}

/* ---- 有符号速度控制 ---- */

void Motor_SetSpeed(const Motor_Cfg *cfg, int32_t speed)
{
    uint32_t duty;
    Motor_Dir dir;

    /* 死区过滤：速度绝对值小于死区 → 停止 */
    if (speed > (int32_t)cfg->deadband) {
        dir  = MOTOR_DIR_FORWARD;
        duty = (uint32_t)speed;
    } else if (speed < -((int32_t)cfg->deadband)) {
        dir  = MOTOR_DIR_BACKWARD;
        duty = (uint32_t)(-speed);
    } else {
        dir  = MOTOR_DIR_STOP;
        duty = 0;
    }

    /* 饱和限幅 */
    if (duty > cfg->pwmPeriod) {
        duty = cfg->pwmPeriod;
    }

    Motor_SetDir(cfg, dir);
    DL_TimerG_setCaptureCompareValue(cfg->pwmInst, duty, cfg->pwmCh);
}

/* ---- 停止 ---- */

void Motor_Stop(const Motor_Cfg *cfg)
{
    Motor_SetDir(cfg, MOTOR_DIR_STOP);
    DL_TimerG_setCaptureCompareValue(cfg->pwmInst, 0, cfg->pwmCh);
}

/* ================================================================
 *  四路电机配置表
 * ================================================================ */

Motor_Cfg gMotor[4] = {
    {   /* motor1: PWM_1 CH0 + AIN1/AIN2 */
        .dir1Port   = DIR_AIN2_PORT,
        .dir1Pin    = DIR_AIN2_PIN,
        .dir2Port   = DIR_AIN1_PORT,
        .dir2Pin    = DIR_AIN1_PIN,
        .pwmInst    = PWM_1_INST,
        .pwmCh      = GPIO_PWM_1_C0_IDX,
        .pwmPeriod  = 1000,
        .deadband   = 10,
    },
    {   /* motor2: PWM_1 CH1 + BIN1/BIN2 */
        .dir1Port   = DIR_BIN2_PORT,
        .dir1Pin    = DIR_BIN2_PIN,
        .dir2Port   = DIR_BIN1_PORT,
        .dir2Pin    = DIR_BIN1_PIN,
        .pwmInst    = PWM_1_INST,
        .pwmCh      = GPIO_PWM_1_C1_IDX,
        .pwmPeriod  = 1000,
        .deadband   = 10,
    },
    {   /* motor3: PWM_0 CH0 + CIN1/CIN2 */
        .dir1Port   = DIR_CIN1_PORT,
        .dir1Pin    = DIR_CIN1_PIN,
        .dir2Port   = DIR_CIN2_PORT,
        .dir2Pin    = DIR_CIN2_PIN,
        .pwmInst    = PWM_0_INST,
        .pwmCh      = GPIO_PWM_0_C0_IDX,
        .pwmPeriod  = 1000,
        .deadband   = 10,
    },
    {   /* motor4: PWM_0 CH1 + DIN1/DIN2 */
        .dir1Port   = DIR_DIN1_PORT,
        .dir1Pin    = DIR_DIN1_PIN,
        .dir2Port   = DIR_DIN2_PORT,
        .dir2Pin    = DIR_DIN2_PIN,
        .pwmInst    = PWM_0_INST,
        .pwmCh      = GPIO_PWM_0_C1_IDX,
        .pwmPeriod  = 1000,
        .deadband   = 10,
    },
};
