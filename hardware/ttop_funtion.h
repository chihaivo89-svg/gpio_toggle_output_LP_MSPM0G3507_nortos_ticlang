/*
 * ttop_funtion.h — 模式执行模块
 *
 * 四个空壳模式函数，后续填充具体逻辑。
 * ModeExecTask() 在 TIMER_0 ISR 槽3 中每 5ms 调用一次。
 */

#ifndef __TTOP_FUNTION_H
#define __TTOP_FUNTION_H

void ModeExecTask(void);

/* 四个模式执行函数（空壳，待实现） */
void Mode1_Task(void);
void Mode2_Task(void);
void Mode3_Task(void);
void Mode4_Task(void);

#endif /* __TTOP_FUNTION_H */
