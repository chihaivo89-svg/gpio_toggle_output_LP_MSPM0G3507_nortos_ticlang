#ifndef __ROAD_TEST_H
#define __ROAD_TEST_H

#include "speed_control.h"

#include <stdbool.h>

/*
 * 脱机直线测试参数：按下 KEY1 后倒计时，四轮向前闭环运行 5 秒。
 * 速度单位仍是编码器每 20ms 的脉冲数，PWM 满量程为 1000。
 */
#define ROAD_TEST_TARGET_PULSES      (5)
#define ROAD_TEST_OUTPUT_LIMIT       (400)
#define ROAD_TEST_COUNTDOWN_MS       (2000UL)
#define ROAD_TEST_DURATION_MS        (5000UL)
#define ROAD_TEST_SAMPLE_PERIOD_MS   (20UL)

typedef enum {
    ROAD_TEST_IDLE = 0,
    ROAD_TEST_COUNTDOWN,
    ROAD_TEST_RUNNING,
    ROAD_TEST_RESULT
} RoadTest_State;

/* 初始化为安全停机状态，不会自动启动电机。 */
void RoadTest_Init(void);

/* 主循环调用：处理 KEY1 边沿、状态切换和 OLED 页面。 */
void RoadTest_Update(const SpeedControl_Status *speedStatus);

/* 每次 20ms 速度环更新后调用，仅把本周期整数数据保存到 RAM。 */
void RoadTest_Record20ms(const SpeedControl_Status *speedStatus);

/* 返回 true 时，主循环应把 OLED 交给脱机测试页面。 */
bool RoadTest_UsesOled(void);

/* 将上一次测试的 RAM 数据按 pid:I0~I5 格式回放给 VOFA。 */
void RoadTest_DumpToVofa(void);

#endif /* __ROAD_TEST_H */
