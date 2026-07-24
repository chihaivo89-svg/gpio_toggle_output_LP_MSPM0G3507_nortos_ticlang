/*
 * nr02_spi.c — 软件 SPI 位操作实现
 *
 * 参考 STM32 示例 User_SPI.c，将 GPIO 位操作替换为 MSPM0 DL 库。
 * 时序：CPOL=0, CPHA=0 (Mode 0)，SCK 空闲低，上升沿采样。
 */

#include "nr02_spi.h"

void SW_SPI_Init(void)
{
    /* SysConfig 已初始化 GPIO 方向，这里只设默认电平 */
    NR02_CE_LOW();
    NR02_SCK_LOW();
    NR02_CSN_HIGH();
}

/*
 * SPI 读写一个字节（MSB first）
 * 发送同时接收，返回收到的字节。
 */
uint8_t SW_SPI_ReadWriteByte(uint8_t byte)
{
    uint8_t i;

    for (i = 0U; i < 8U; i++) {
        /* MOSI = 当前字节的最高位 */
        NR02_MOSI_WRITE((byte & 0x80U) >> 7);
        byte <<= 1U;

        /* SCK 上升沿 → 从机采样 MOSI、从机输出 MISO */
        NR02_SCK_HIGH();

        /* 读 MISO */
        byte |= NR02_MISO_READ();

        /* SCK 下降沿 */
        NR02_SCK_LOW();
    }

    return byte;
}
