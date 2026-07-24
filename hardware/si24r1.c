/*
 * si24r1.c — Si24R1 (NRF24L01 兼容) 底层驱动实现
 *
 * 主要改动（相对 STM32 原版）：
 *   1. GPIO 操作替换为 nr02_spi.h 内联函数
 *   2. 去 IRQ 引脚依赖：TxPacket 改为轮询 STATUS + 超时
 *   3. 数据类型 uint8_t，延时使用工程 Delay_ms/Delay_us
 */

#include "si24r1.h"
#include "nr02_spi.h"
#include "delay.h"

/* 默认收发地址（5 字节） */
static const uint8_t s_txAddr[TX_ADDR_WIDTH] = {0x01, 0x02, 0x03, 0x04, 0x05};
static const uint8_t s_rxAddr[RX_ADDR_WIDTH] = {0x01, 0x02, 0x03, 0x04, 0x05};

/* ================================================================
 *  底层 SPI 读写
 * ================================================================ */

uint8_t Si24R1_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    NR02_CSN_LOW();
    status = SW_SPI_ReadWriteByte(reg);
    SW_SPI_ReadWriteByte(value);
    NR02_CSN_HIGH();
    return status;
}

uint8_t Si24R1_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    NR02_CSN_LOW();
    SW_SPI_ReadWriteByte(reg);
    reg_val = SW_SPI_ReadWriteByte(NOP);
    NR02_CSN_HIGH();
    return reg_val;
}

uint8_t Si24R1_Write_Buf(uint8_t reg, const uint8_t *pBuf, uint8_t len)
{
    uint8_t status;
    uint8_t ctr;
    NR02_CSN_LOW();
    status = SW_SPI_ReadWriteByte(reg);
    for (ctr = 0U; ctr < len; ctr++) {
        SW_SPI_ReadWriteByte(pBuf[ctr]);
    }
    NR02_CSN_HIGH();
    return status;
}

uint8_t Si24R1_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status;
    uint8_t ctr;
    NR02_CSN_LOW();
    status = SW_SPI_ReadWriteByte(reg);
    for (ctr = 0U; ctr < len; ctr++) {
        pBuf[ctr] = SW_SPI_ReadWriteByte(0xFFU);
    }
    NR02_CSN_HIGH();
    return status;
}

/* ================================================================
 *  收发模式切换
 * ================================================================ */

void Si24R1_TX_Mode(void)
{
    NR02_CE_LOW();
    Si24R1_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0EU);
    Si24R1_Write_Reg(FLUSH_RX, 0xFFU);
    Si24R1_Write_Reg(FLUSH_TX, 0xFFU);
    NR02_CE_HIGH();
}

void Si24R1_RX_Mode(void)
{
    NR02_CE_LOW();
    Si24R1_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0FU);
    Si24R1_Write_Reg(FLUSH_RX, 0xFFU);
    Si24R1_Write_Reg(FLUSH_TX, 0xFFU);
    NR02_CE_HIGH();
}

/* ================================================================
 *  收发数据包
 * ================================================================ */

uint8_t Si24R1_TxPacket(uint8_t *txbuf)
{
    uint8_t sta;
    uint16_t timeout;

    NR02_CE_LOW();
    Si24R1_TX_Mode();
    Si24R1_Write_Buf(WR_TX_PLOAD, txbuf, W_RX_PAYLOAD_WIDTH);
    NR02_CE_HIGH();

    /*
     * 轮询 STATUS 寄存器，等待 TX_DS 或 MAX_TX 置位。
     * 不依赖 IRQ 引脚，避免硬件连接限制。
     */
    timeout = 0U;
    do {
        sta = Si24R1_Read_Reg(NRF_STATUS);
        if (sta & (TX_OK | MAX_TX)) {
            break;
        }
        Delay_us(10);
        timeout++;
    } while (timeout < 5000U);  /* 约 50ms 超时 */

    /* 清除中断标志 */
    Si24R1_Write_Reg(NRF_WRITE_REG + NRF_STATUS, sta);

    /* 切回 RX 模式等待应答或后续指令 */
    Si24R1_RX_Mode();

    if (sta & MAX_TX) {
        Si24R1_Write_Reg(FLUSH_TX, 0xFFU);
        return 2U;  /* 达到最大重发次数 */
    }

    if (sta & TX_OK) {
        return 1U;  /* 发送成功 */
    }

    return 0U;      /* 超时或未知错误 */
}

uint8_t Si24R1_RxPacket(uint8_t *rxbuf)
{
    uint8_t sta;

    sta = Si24R1_Read_Reg(NRF_STATUS);
    Si24R1_Write_Reg(NRF_WRITE_REG + NRF_STATUS, sta);

    if (sta & RX_OK) {
        Si24R1_Read_Buf(RD_RX_PLOAD, rxbuf, R_RX_PAYLOAD_WIDTH);
        Si24R1_Write_Reg(FLUSH_RX, 0xFFU);
        return 0U;  /* 成功 */
    }

    return 1U;      /* 无数据 */
}

/* ================================================================
 *  检测与初始化
 * ================================================================ */

uint8_t Si24R1_Check(void)
{
    uint8_t check_in[5]  = {0x55, 0xAA, 0x55, 0xAA, 0x55};
    uint8_t check_out[5] = {0};

    NR02_CE_LOW();
    Si24R1_Write_Buf(NRF_WRITE_REG + TX_ADDR, check_in, 5);
    Si24R1_Read_Buf(NRF_READ_REG + TX_ADDR, check_out, 5);

    if (check_out[0] == 0x55 && check_out[1] == 0xAA &&
        check_out[2] == 0x55 && check_out[3] == 0xAA &&
        check_out[4] == 0x55) {
        return 0U;  /* 检测成功 */
    }
    return 1U;      /* 检测失败 */
}

void Si24R1_Init(void)
{
    /* 等待模块就绪 */
    while (Si24R1_Check()) {
        Delay_ms(500);
    }

    NR02_CE_LOW();
    Si24R1_Write_Reg(NRF_WRITE_REG + RX_PW_P0, R_RX_PAYLOAD_WIDTH);
    Si24R1_Write_Buf(NRF_WRITE_REG + TX_ADDR, s_txAddr, TX_ADDR_WIDTH);
    Si24R1_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, s_rxAddr, RX_ADDR_WIDTH);
    Si24R1_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01U);        /* 使能自动应答（通道 0） */
    Si24R1_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01U);   /* 使能通道 0 接收地址 */
    Si24R1_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x03U);  /* 250us 重发间隔，自动重发 3 次 */
    Si24R1_Write_Reg(NRF_WRITE_REG + RF_CH, 2U);           /* 2402 MHz */
    Si24R1_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0FU);    /* 2 Mbps, 7 dBm */
    Si24R1_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0FU);      /* PWR_UP, EN_CRC, 16BIT_CRC, RX 模式 */
    NR02_CE_HIGH();
}

/* ================================================================
 *  快捷初始化
 * ================================================================ */

void RF_RX_Init(void)
{
    SW_SPI_Init();
    Si24R1_Init();
    Si24R1_RX_Mode();
}

void RF_TX_Init(void)
{
    SW_SPI_Init();
    Si24R1_Init();
    Si24R1_TX_Mode();
}
