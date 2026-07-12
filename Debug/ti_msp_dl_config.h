/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMG8
#define PWM_0_INST_IRQHandler                                   TIMG8_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMG8_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                         DL_GPIO_PIN_21
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM46)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM46_PF_TIMG8_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOA
#define GPIO_PWM_0_C1_PIN                                         DL_GPIO_PIN_27
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM60)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM60_PF_TIMG8_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX

/* Defines for PWM_1 */
#define PWM_1_INST                                                         TIMG6
#define PWM_1_INST_IRQHandler                                   TIMG6_IRQHandler
#define PWM_1_INST_INT_IRQN                                     (TIMG6_INT_IRQn)
#define PWM_1_INST_CLK_FREQ                                             32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_1_C0_PORT                                                 GPIOB
#define GPIO_PWM_1_C0_PIN                                         DL_GPIO_PIN_26
#define GPIO_PWM_1_C0_IOMUX                                      (IOMUX_PINCM57)
#define GPIO_PWM_1_C0_IOMUX_FUNC                     IOMUX_PINCM57_PF_TIMG6_CCP0
#define GPIO_PWM_1_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_1_C1_PORT                                                 GPIOB
#define GPIO_PWM_1_C1_PIN                                         DL_GPIO_PIN_27
#define GPIO_PWM_1_C1_IOMUX                                      (IOMUX_PINCM58)
#define GPIO_PWM_1_C1_IOMUX_FUNC                     IOMUX_PINCM58_PF_TIMG6_CCP1
#define GPIO_PWM_1_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for E1_B */
#define E1_B_INST                                                        (TIMG0)
#define E1_B_INST_IRQHandler                                    TIMG0_IRQHandler
#define E1_B_INST_INT_IRQN                                      (TIMG0_INT_IRQn)
#define E1_B_INST_LOAD_VALUE                                            (31999U)
/* GPIO defines for channel 0 */
#define GPIO_E1_B_C0_PORT                                                  GPIOA
#define GPIO_E1_B_C0_PIN                                          DL_GPIO_PIN_12
#define GPIO_E1_B_C0_IOMUX                                       (IOMUX_PINCM34)
#define GPIO_E1_B_C0_IOMUX_FUNC                      IOMUX_PINCM34_PF_TIMG0_CCP0

/* Defines for E3_B */
#define E3_B_INST                                                        (TIMA1)
#define E3_B_INST_IRQHandler                                    TIMA1_IRQHandler
#define E3_B_INST_INT_IRQN                                      (TIMA1_INT_IRQn)
#define E3_B_INST_LOAD_VALUE                                            (31999U)
/* GPIO defines for channel 0 */
#define GPIO_E3_B_C0_PORT                                                  GPIOB
#define GPIO_E3_B_C0_PIN                                           DL_GPIO_PIN_4
#define GPIO_E3_B_C0_IOMUX                                       (IOMUX_PINCM17)
#define GPIO_E3_B_C0_IOMUX_FUNC                      IOMUX_PINCM17_PF_TIMA1_CCP0





/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMA0)
#define TIMER_0_INST_IRQHandler                                 TIMA0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMA0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                         (31999U)




/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C1
#define I2C_OLED_INST_IRQHandler                                 I2C1_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C1_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOA
#define GPIO_I2C_OLED_SDA_PIN                                     DL_GPIO_PIN_16
#define GPIO_I2C_OLED_IOMUX_SDA                                  (IOMUX_PINCM38)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                   IOMUX_PINCM38_PF_I2C1_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOA
#define GPIO_I2C_OLED_SCL_PIN                                     DL_GPIO_PIN_15
#define GPIO_I2C_OLED_IOMUX_SCL                                  (IOMUX_PINCM37)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                   IOMUX_PINCM37_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART1
#define UART_0_INST_FREQUENCY                                           32000000
#define UART_0_INST_IRQHandler                                  UART1_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                         DL_GPIO_PIN_9
#define GPIO_UART_0_TX_PIN                                         DL_GPIO_PIN_8
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM20)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM19)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM20_PF_UART1_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM19_PF_UART1_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_0_FBRD_32_MHZ_115200_BAUD                                      (23)




/* Defines for SPI_IMU660RB */
#define SPI_IMU660RB_INST                                                  SPI1
#define SPI_IMU660RB_INST_IRQHandler                            SPI1_IRQHandler
#define SPI_IMU660RB_INST_INT_IRQN                                SPI1_INT_IRQn
#define GPIO_SPI_IMU660RB_PICO_PORT                                       GPIOB
#define GPIO_SPI_IMU660RB_PICO_PIN                               DL_GPIO_PIN_22
#define GPIO_SPI_IMU660RB_IOMUX_PICO                            (IOMUX_PINCM50)
#define GPIO_SPI_IMU660RB_IOMUX_PICO_FUNC            IOMUX_PINCM50_PF_SPI1_PICO
#define GPIO_SPI_IMU660RB_POCI_PORT                                       GPIOB
#define GPIO_SPI_IMU660RB_POCI_PIN                               DL_GPIO_PIN_14
#define GPIO_SPI_IMU660RB_IOMUX_POCI                            (IOMUX_PINCM31)
#define GPIO_SPI_IMU660RB_IOMUX_POCI_FUNC            IOMUX_PINCM31_PF_SPI1_POCI
/* GPIO configuration for SPI_IMU660RB */
#define GPIO_SPI_IMU660RB_SCLK_PORT                                       GPIOA
#define GPIO_SPI_IMU660RB_SCLK_PIN                               DL_GPIO_PIN_17
#define GPIO_SPI_IMU660RB_IOMUX_SCLK                            (IOMUX_PINCM39)
#define GPIO_SPI_IMU660RB_IOMUX_SCLK_FUNC            IOMUX_PINCM39_PF_SPI1_SCLK



/* Port definition for Pin Group GPIO_IMU660RB */
#define GPIO_IMU660RB_PORT                                               (GPIOB)

/* Defines for PIN_IMU660RB_CS: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GPIO_IMU660RB_PIN_IMU660RB_CS_PIN                        (DL_GPIO_PIN_0)
#define GPIO_IMU660RB_PIN_IMU660RB_CS_IOMUX                      (IOMUX_PINCM12)
/* Defines for E1_A: GPIOA.13 with pinCMx 35 on package pin 6 */
#define ECO_E1_A_PORT                                                    (GPIOA)
#define ECO_E1_A_PIN                                            (DL_GPIO_PIN_13)
#define ECO_E1_A_IOMUX                                           (IOMUX_PINCM35)
/* Defines for E3_A: GPIOB.5 with pinCMx 18 on package pin 53 */
#define ECO_E3_A_PORT                                                    (GPIOB)
#define ECO_E3_A_PIN                                             (DL_GPIO_PIN_5)
#define ECO_E3_A_IOMUX                                           (IOMUX_PINCM18)
/* Defines for AIN1: GPIOB.13 with pinCMx 30 on package pin 1 */
#define DIR_AIN1_PORT                                                    (GPIOB)
#define DIR_AIN1_PIN                                            (DL_GPIO_PIN_13)
#define DIR_AIN1_IOMUX                                           (IOMUX_PINCM30)
/* Defines for AIN2: GPIOA.14 with pinCMx 36 on package pin 7 */
#define DIR_AIN2_PORT                                                    (GPIOA)
#define DIR_AIN2_PIN                                            (DL_GPIO_PIN_14)
#define DIR_AIN2_IOMUX                                           (IOMUX_PINCM36)
/* Defines for BIN1: GPIOB.18 with pinCMx 44 on package pin 15 */
#define DIR_BIN1_PORT                                                    (GPIOB)
#define DIR_BIN1_PIN                                            (DL_GPIO_PIN_18)
#define DIR_BIN1_IOMUX                                           (IOMUX_PINCM44)
/* Defines for BIN2: GPIOB.19 with pinCMx 45 on package pin 16 */
#define DIR_BIN2_PORT                                                    (GPIOB)
#define DIR_BIN2_PIN                                            (DL_GPIO_PIN_19)
#define DIR_BIN2_IOMUX                                           (IOMUX_PINCM45)
/* Defines for CIN1: GPIOA.25 with pinCMx 55 on package pin 26 */
#define DIR_CIN1_PORT                                                    (GPIOA)
#define DIR_CIN1_PIN                                            (DL_GPIO_PIN_25)
#define DIR_CIN1_IOMUX                                           (IOMUX_PINCM55)
/* Defines for CIN2: GPIOB.25 with pinCMx 56 on package pin 27 */
#define DIR_CIN2_PORT                                                    (GPIOB)
#define DIR_CIN2_PIN                                            (DL_GPIO_PIN_25)
#define DIR_CIN2_IOMUX                                           (IOMUX_PINCM56)
/* Defines for DIN1: GPIOA.1 with pinCMx 2 on package pin 34 */
#define DIR_DIN1_PORT                                                    (GPIOA)
#define DIR_DIN1_PIN                                             (DL_GPIO_PIN_1)
#define DIR_DIN1_IOMUX                                            (IOMUX_PINCM2)
/* Defines for DIN2: GPIOA.26 with pinCMx 59 on package pin 30 */
#define DIR_DIN2_PORT                                                    (GPIOA)
#define DIR_DIN2_PIN                                            (DL_GPIO_PIN_26)
#define DIR_DIN2_IOMUX                                           (IOMUX_PINCM59)
/* Defines for IMU: GPIOB.1 with pinCMx 13 on package pin 48 */
#define TEST_IMU_PORT                                                    (GPIOB)
#define TEST_IMU_PIN                                             (DL_GPIO_PIN_1)
#define TEST_IMU_IOMUX                                           (IOMUX_PINCM13)
/* Defines for Fusion: GPIOA.2 with pinCMx 7 on package pin 42 */
#define TEST_Fusion_PORT                                                 (GPIOA)
#define TEST_Fusion_PIN                                          (DL_GPIO_PIN_2)
#define TEST_Fusion_IOMUX                                         (IOMUX_PINCM7)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_PWM_1_init(void);
void SYSCFG_DL_E1_B_init(void);
void SYSCFG_DL_E3_B_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_SPI_IMU660RB_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
