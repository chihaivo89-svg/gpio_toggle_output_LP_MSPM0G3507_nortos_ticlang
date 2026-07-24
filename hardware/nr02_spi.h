/*
 * nr02_spi.h — 软件 SPI（用于 DX-NR02 / Si24R1 2.4G 模块）
 *
 * 引脚映射（SysConfig USER_SPI 组）：
 *   CS/N  = PB12  (OUTPUT)
 *   CE    = PB6   (OUTPUT)
 *   SCK   = PB8   (OUTPUT)
 *   MOSI  = PB7   (OUTPUT)
 *   MISO  = PB9   (INPUT)
 */

#ifndef __NR02_SPI_H
#define __NR02_SPI_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ---- 引脚宏 ---- */
#define NR02_PORT       (GPIOB)

#define NR02_CSN_PIN    (DL_GPIO_PIN_12)
#define NR02_CE_PIN     (DL_GPIO_PIN_6)
#define NR02_SCK_PIN    (DL_GPIO_PIN_8)
#define NR02_MOSI_PIN   (DL_GPIO_PIN_7)
#define NR02_MISO_PIN   (DL_GPIO_PIN_9)

/* ---- GPIO 位操作内联函数 ---- */
static inline void NR02_CSN_LOW(void)  { DL_GPIO_clearPins(NR02_PORT, NR02_CSN_PIN); }
static inline void NR02_CSN_HIGH(void) { DL_GPIO_setPins(NR02_PORT, NR02_CSN_PIN); }

static inline void NR02_CE_LOW(void)   { DL_GPIO_clearPins(NR02_PORT, NR02_CE_PIN); }
static inline void NR02_CE_HIGH(void)  { DL_GPIO_setPins(NR02_PORT, NR02_CE_PIN); }

static inline void NR02_SCK_LOW(void)  { DL_GPIO_clearPins(NR02_PORT, NR02_SCK_PIN); }
static inline void NR02_SCK_HIGH(void) { DL_GPIO_setPins(NR02_PORT, NR02_SCK_PIN); }

static inline void NR02_MOSI_WRITE(uint8_t bit)
{
    if (bit) {
        DL_GPIO_setPins(NR02_PORT, NR02_MOSI_PIN);
    } else {
        DL_GPIO_clearPins(NR02_PORT, NR02_MOSI_PIN);
    }
}

static inline uint8_t NR02_MISO_READ(void)
{
    return (DL_GPIO_readPins(NR02_PORT, NR02_MISO_PIN) != 0U) ? 1U : 0U;
}

/* ---- API ---- */

void SW_SPI_Init(void);
uint8_t SW_SPI_ReadWriteByte(uint8_t byte);

#endif /* __NR02_SPI_H */
