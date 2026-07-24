/*
 * param_storage.h — 4 套模式参数存储（EEPROM Type A）
 *
 * 每次保存会将全部 4 套参数写入 EEPROM。
 * 上电自动读取，CRC 校验失败回退默认值。
 */

#ifndef __PARAM_STORAGE_H
#define __PARAM_STORAGE_H

#include <stdint.h>

#define PARAM_MODE_COUNT    (4U)

/* 每套参数结构（11 × uint32_t = 44 字节） */
typedef struct {
    float   speedKp;
    float   speedKi;
    float   speedKd;
    int16_t speedOutLimit;

    float   trackKp;
    float   trackKi;
    float   trackKd;
    float   trackOutMax;
    float   trackIMax;
    int16_t trackBaseSpeed;
    int16_t trackTurnSpeed;   /* 正方形循迹转弯速度（脉冲/20ms） */

    int16_t entryEnable;      /* 直角入口等待使能（0=关闭, 1=开启） */
    int16_t entryThreshold;   /* 入口等待编码器增量阈值（默认 200） */
    int16_t stopAfterTurns;   /* 运行 x 个直角后停止（0=不限制） */
} ModeParams;  /* 11 × uint32_t = 44 字节 */

/* 全部 4 套参数 */
extern ModeParams g_modeParams[PARAM_MODE_COUNT];

/* ---- API ---- */

/* 上电初始化：从 EEPROM 加载或填充默认值 */
void Param_Init(void);

/* 将 g_modeParams[] 写入 EEPROM */
void Param_SaveAll(void);

/* 将某个 mode 的参数应用到运行时 PID 变量 */
void Param_ApplyMode(uint8_t mode);

#endif /* __PARAM_STORAGE_H */
