/*
 * param_storage.c — 4 套模式参数存储实现
 *
 * EEPROM 缓冲区布局（62 × uint32_t = 248 字节）:
 *   [0]        = VERSION (1)
 *   [1..11]    = ModeParams[0] (11 uint32_t)
 *   [12..22]   = ModeParams[1]
 *   [23..33]   = ModeParams[2]
 *   [34..44]   = ModeParams[3]
 *   [45..46]   = CRC16 (2 uint32_t)
 *   剩余保留
 */

#include "param_storage.h"
#include "eeprom_emulation_type_a.h"
#include "speed_control.h"
#include "track.h"

/* ---- 全局参数数组 ---- */
ModeParams g_modeParams[PARAM_MODE_COUNT];

/* ---- EEPROM 缓冲区大小 ---- */
#define EEP_BUF_LEN  (EEPROM_EMULATION_DATA_SIZE / sizeof(uint32_t))  /* 62 */

#define PARAM_VERSION  (1U)

/* 序列化/反序列化每个 ModeParams 使用的 uint32_t 个数 */
#define PARAM_U32_PER_MODE  (sizeof(ModeParams) / sizeof(uint32_t))  /* 11 */

/* ---- 默认值（上电/校验失败时使用） ---- */
static const ModeParams s_defaultParams = {
    .speedKp        = 30.0f,
    .speedKi        = 1.5f,
    .speedKd        = 0.0f,
    .speedOutLimit  = 800,
    .trackKp        = 1.0f,
    .trackKi        = 0.0f,
    .trackKd        = 0.0f,
    .trackOutMax    = 15.0f,
    .trackIMax      = 30.0f,
    .trackBaseSpeed = 12,
    .trackTurnSpeed = 6,
    .entryEnable    = 0,        /* 默认关闭 */
    .entryThreshold = 200,      /* 默认 200 脉冲 */
    .stopAfterTurns = 0,        /* 0 = 不限制 */
};

/* ---- 简单 CRC16（多项式 0x8005） ---- */
static uint16_t Param_CalcCrc(const uint32_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        uint32_t d = buf[i];
        for (int b = 0; b < 32; b++) {
            uint16_t bit = (uint16_t)((crc >> 15) ^ (d >> 31));
            crc <<= 1;
            if (bit) crc ^= 0x8005;
            d <<= 1;
        }
    }
    return crc;
}

/* ---- 序列化 ModeParams → uint32_t[10] ---- */
static void PackParams(uint32_t *buf, const ModeParams *p)
{
    const uint32_t *src = (const uint32_t *)p;
    for (int i = 0; i < PARAM_U32_PER_MODE; i++) {
        buf[i] = src[i];
    }
}

/* ---- 反序列化 uint32_t[10] → ModeParams ---- */
static void UnpackParams(ModeParams *p, const uint32_t *buf)
{
    uint32_t *dst = (uint32_t *)p;
    for (int i = 0; i < PARAM_U32_PER_MODE; i++) {
        dst[i] = buf[i];
    }
}

/* ---- 填充默认值 ---- */

static void Param_SetDefaults(void)
{
    for (uint8_t m = 0; m < PARAM_MODE_COUNT; m++) {
        g_modeParams[m] = s_defaultParams;
    }
}

/* ---- 公开 API ---- */

void Param_Init(void)
{
    uint32_t buf[EEP_BUF_LEN];

    Param_SetDefaults();

    if (EEPROM_TypeA_init(buf) != EEPROM_EMULATION_INIT_OK) {
        /* Flash 初始化失败（首次冷态），保持默认值 */
        return;
    }

    uint16_t crcLen = 1U + PARAM_MODE_COUNT * PARAM_U32_PER_MODE;

    /* 校验版本和 CRC */
    if (buf[0] != PARAM_VERSION) {
        return;  /* 版本不匹配，保留默认值 */
    }

    uint16_t savedCrc = (uint16_t)(buf[crcLen] & 0xFFFF);
    uint16_t calcedCrc = Param_CalcCrc(buf, crcLen);
    if (savedCrc != calcedCrc) {
        return;  /* CRC 不匹配，保留默认值 */
    }

    /* 反序列化 */
    for (uint8_t m = 0; m < PARAM_MODE_COUNT; m++) {
        UnpackParams(&g_modeParams[m], &buf[1 + m * PARAM_U32_PER_MODE]);
    }
}

void Param_SaveAll(void)
{
    uint32_t buf[EEP_BUF_LEN];

    buf[0] = PARAM_VERSION;

    for (uint8_t m = 0; m < PARAM_MODE_COUNT; m++) {
        PackParams(&buf[1 + m * PARAM_U32_PER_MODE], &g_modeParams[m]);
    }

    uint16_t crcLen = 1U + PARAM_MODE_COUNT * PARAM_U32_PER_MODE;

    /* 清零保留区 */
    for (uint16_t i = crcLen; i < EEP_BUF_LEN; i++) {
        buf[i] = 0;
    }

    /* CRC */
    uint16_t crc = Param_CalcCrc(buf, crcLen);
    buf[crcLen]     = crc;
    buf[crcLen + 1] = 0;

    (void)EEPROM_TypeA_writeData(buf);
}

void Param_ApplyMode(uint8_t mode)
{
    if (mode < 1 || mode > PARAM_MODE_COUNT) return;
    const ModeParams *p = &g_modeParams[mode - 1];

    SpeedControl_SetKp(p->speedKp);
    SpeedControl_SetKi(p->speedKi);
    SpeedControl_SetKd(p->speedKd);
    SpeedControl_SetOutLimit(p->speedOutLimit);

    Track_SetKp(p->trackKp);
    Track_SetKi(p->trackKi);
    Track_SetKd(p->trackKd);
    Track_SetOutMax(p->trackOutMax);
    Track_SetIMax(p->trackIMax);
    Track_SetBaseSpeed(p->trackBaseSpeed);
}
