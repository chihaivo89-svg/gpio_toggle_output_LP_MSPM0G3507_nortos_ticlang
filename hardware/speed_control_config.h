/*
 * speed_control_config.h - 正式 V2 速度环参数调整入口
 *
 * 常规调参只修改本文件，不要在 speed_control.c 中散落参数。
 *
 * 单位说明：
 *   - target / actual：每 20ms 的编码器脉冲数
 *   - output / limit：PWM 计数值，电机底层满量程为 1000
 *   - trim：千分比，例如 75 表示 7.5%
 *
 * 运行目标速度不是固定宏，由上层在运行前设置：
 *   SpeedControl_SetTargets(18, 18);
 *   SpeedControl_Start();
 *
 * 已验证范围为 Target 8~20。超出该范围前，应重新做架空、负载和地面测试。
 */

#ifndef SPEED_CONTROL_CONFIG_H
#define SPEED_CONTROL_CONFIG_H

/* ==================== 常用调参区 ==================== */

/*
 * 速度 PI/PID 参数。
 * Ki 已包含 20ms 离散控制周期；当前 Kd=0，实际运行的是 PI 控制。
 * 建议调节顺序：Kp -> Ki -> Kd，且每次只改变一个参数。
 */
#define SPEED_PID_KP                       (30.0f)
#define SPEED_PID_KI                       (1.5f)
#define SPEED_PID_KD                       (0.0f)

/*
 * PID 最终输出限制。800 表示最多使用 80% PWM。
 * 修改后需要重新检查堵转电流、驱动温度和输出饱和情况。
 */
#define SPEED_OUTPUT_LIMIT                 (800)

/*
 * 无编码器前轮的静态补偿：
 *   M4 左前轮降低 7.5%
 *   M2 右前轮提高 7.5%
 * 只在左右两侧均向前运行时生效。
 */
#define SPEED_M4_TRIM_PERMILLE             (-75)
#define SPEED_M2_TRIM_PERMILLE             (75)

/* ==================== 安全边界 ==================== */

/*
 * SetTargets() 的输入保护范围，不代表已经验证到 Target 200。
 * 当前正式验证范围仍是 Target 8~20。
 */
#define SPEED_MAX_ABS_TARGET               (200)

/* ==================== V2 高级参数 ==================== */

/*
 * 以下参数决定目标斜坡、前馈和抗积分饱和行为。
 * 常规 PID 微调不需要修改；改变后应重新完成整套标准化测试。
 */
#define SPEED_TARGET_STEP                  (1)
#define SPEED_FF_LINEAR_TARGET             (8)
#define SPEED_FF_MAX_TARGET                (20)
#define SPEED_FF_OFFSET                    (198.762f)
#define SPEED_FF_GAIN                      (11.340f)
#define SPEED_FF_MAX_PERMILLE              (750)
#define SPEED_ANTI_WINDUP_GAIN             (0.25f)
#define SPEED_INTEGRAL_UNLOAD_SCALE        (3.0f)
#define SPEED_UNLOAD_MIN_ERROR             (2)

/* trim 的固定换算基数，不作为调参项。 */
#define SPEED_TRIM_SCALE_BASE              (1000)

/* 参数越过电机 PWM 满量程时直接阻止编译，避免误配置进入硬件。 */
#if (SPEED_OUTPUT_LIMIT < 0) || (SPEED_OUTPUT_LIMIT > 1000)
#error "SPEED_OUTPUT_LIMIT must be in range 0..1000"
#endif

#endif /* SPEED_CONTROL_CONFIG_H */
