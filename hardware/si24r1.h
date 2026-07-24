/*
 * si24r1.h — Si24R1 (NRF24L01 兼容) 2.4G 无线模块驱动
 *
 * 移植自 DX-NR02 STM32 示例，去除 IRQ 引脚依赖，改用轮询 + 超时。
 * 软件 SPI 层通过 nr02_spi.h 提供。
 */

#ifndef __SI24R1_H
#define __SI24R1_H

#include <stdint.h>

#define TX_ADDR_WIDTH       5
#define RX_ADDR_WIDTH       5
#define W_RX_PAYLOAD_WIDTH  32
#define R_RX_PAYLOAD_WIDTH  32

/* ---- 命令 ---- */
#define NRF_READ_REG    0x00U
#define NRF_WRITE_REG   0x20U
#define RD_RX_PLOAD     0x61U
#define WR_TX_PLOAD     0xA0U
#define FLUSH_TX        0xE1U
#define FLUSH_RX        0xE2U
#define REUSE_TX_PL     0xE3U
#define NOP             0xFFU

/* ---- 寄存器地址 ---- */
#define CONFIG          0x00U
#define EN_AA           0x01U
#define EN_RXADDR       0x02U
#define SETUP_AW        0x03U
#define SETUP_RETR      0x04U
#define RF_CH           0x05U
#define RF_SETUP        0x06U
#define NRF_STATUS      0x07U
#define OBSERVE_TX      0x08U
#define CD              0x09U
#define RX_ADDR_P0      0x0AU
#define RX_ADDR_P1      0x0BU
#define RX_ADDR_P2      0x0CU
#define RX_ADDR_P3      0x0DU
#define RX_ADDR_P4      0x0EU
#define RX_ADDR_P5      0x0FU
#define TX_ADDR         0x10U
#define RX_PW_P0        0x11U
#define RX_PW_P1        0x12U
#define RX_PW_P2        0x13U
#define RX_PW_P3        0x14U
#define RX_PW_P4        0x15U
#define RX_PW_P5        0x16U
#define NRF_FIFO_STATUS 0x17U

/* ---- STATUS 标志位 ---- */
#define MAX_TX          0x10U
#define TX_OK           0x20U
#define RX_OK           0x40U

/* ---- API ---- */

uint8_t Si24R1_Write_Reg(uint8_t reg, uint8_t value);
uint8_t Si24R1_Read_Reg(uint8_t reg);
uint8_t Si24R1_Write_Buf(uint8_t reg, const uint8_t *pBuf, uint8_t len);
uint8_t Si24R1_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);

uint8_t Si24R1_Check(void);
void    Si24R1_Init(void);

void   Si24R1_TX_Mode(void);
void   Si24R1_RX_Mode(void);
uint8_t Si24R1_TxPacket(uint8_t *txbuf);
uint8_t Si24R1_RxPacket(uint8_t *rxbuf);

void RF_RX_Init(void);
void RF_TX_Init(void);

#endif /* __SI24R1_H */
