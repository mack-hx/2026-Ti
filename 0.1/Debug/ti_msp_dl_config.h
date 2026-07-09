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



#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2500
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for TB6612_PWM */
#define TB6612_PWM_INST                                                   TIMG12
#define TB6612_PWM_INST_IRQHandler                             TIMG12_IRQHandler
#define TB6612_PWM_INST_INT_IRQN                               (TIMG12_INT_IRQn)
#define TB6612_PWM_INST_CLK_FREQ                                        40000000
/* GPIO defines for channel 0 */
#define GPIO_TB6612_PWM_C0_PORT                                            GPIOB
#define GPIO_TB6612_PWM_C0_PIN                                    DL_GPIO_PIN_13
#define GPIO_TB6612_PWM_C0_IOMUX                                 (IOMUX_PINCM30)
#define GPIO_TB6612_PWM_C0_IOMUX_FUNC               IOMUX_PINCM30_PF_TIMG12_CCP0
#define GPIO_TB6612_PWM_C0_IDX                               DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_TB6612_PWM_C1_PORT                                            GPIOA
#define GPIO_TB6612_PWM_C1_PIN                                    DL_GPIO_PIN_25
#define GPIO_TB6612_PWM_C1_IOMUX                                 (IOMUX_PINCM55)
#define GPIO_TB6612_PWM_C1_IOMUX_FUNC               IOMUX_PINCM55_PF_TIMG12_CCP1
#define GPIO_TB6612_PWM_C1_IDX                               DL_TIMER_CC_1_INDEX



/* Defines for BIANMA1_TIM */
#define BIANMA1_TIM_INST                                                 (TIMG7)
#define BIANMA1_TIM_INST_IRQHandler                             TIMG7_IRQHandler
#define BIANMA1_TIM_INST_INT_IRQN                               (TIMG7_INT_IRQn)
#define BIANMA1_TIM_INST_LOAD_VALUE                                     (49999U)
/* Defines for BIANMA2_TIM */
#define BIANMA2_TIM_INST                                                 (TIMA1)
#define BIANMA2_TIM_INST_IRQHandler                             TIMA1_IRQHandler
#define BIANMA2_TIM_INST_INT_IRQN                               (TIMA1_INT_IRQn)
#define BIANMA2_TIM_INST_LOAD_VALUE                                     (49999U)
/* Defines for SYSTEM_TIIM */
#define SYSTEM_TIIM_INST                                                 (TIMA0)
#define SYSTEM_TIIM_INST_IRQHandler                             TIMA0_IRQHandler
#define SYSTEM_TIIM_INST_INT_IRQN                               (TIMA0_INT_IRQn)
#define SYSTEM_TIIM_INST_LOAD_VALUE                                      (9999U)




/* Defines for MPU9250 */
#define MPU9250_INST                                                        I2C0
#define MPU9250_INST_IRQHandler                                  I2C0_IRQHandler
#define MPU9250_INST_INT_IRQN                                      I2C0_INT_IRQn
#define MPU9250_BUS_SPEED_HZ                                              400000
#define GPIO_MPU9250_SDA_PORT                                              GPIOA
#define GPIO_MPU9250_SDA_PIN                                       DL_GPIO_PIN_0
#define GPIO_MPU9250_IOMUX_SDA                                    (IOMUX_PINCM1)
#define GPIO_MPU9250_IOMUX_SDA_FUNC                     IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_MPU9250_SCL_PORT                                              GPIOA
#define GPIO_MPU9250_SCL_PIN                                       DL_GPIO_PIN_1
#define GPIO_MPU9250_IOMUX_SCL                                    (IOMUX_PINCM2)
#define GPIO_MPU9250_IOMUX_SCL_FUNC                     IOMUX_PINCM2_PF_I2C0_SCL

/* Defines for I2C_1 */
#define I2C_1_INST                                                          I2C1
#define I2C_1_INST_IRQHandler                                    I2C1_IRQHandler
#define I2C_1_INST_INT_IRQN                                        I2C1_INT_IRQn
#define GPIO_I2C_1_SDA_PORT                                                GPIOA
#define GPIO_I2C_1_SDA_PIN                                        DL_GPIO_PIN_30
#define GPIO_I2C_1_IOMUX_SDA                                      (IOMUX_PINCM5)
#define GPIO_I2C_1_IOMUX_SDA_FUNC                       IOMUX_PINCM5_PF_I2C1_SDA
#define GPIO_I2C_1_SCL_PORT                                                GPIOA
#define GPIO_I2C_1_SCL_PIN                                        DL_GPIO_PIN_17
#define GPIO_I2C_1_IOMUX_SCL                                     (IOMUX_PINCM39)
#define GPIO_I2C_1_IOMUX_SCL_FUNC                      IOMUX_PINCM39_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           40000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_40_MHZ_9600_BAUD                                       (260)
#define UART_0_FBRD_40_MHZ_9600_BAUD                                        (27)
/* Defines for K230 */
#define K230_INST                                                          UART1
#define K230_INST_FREQUENCY                                             40000000
#define K230_INST_IRQHandler                                    UART1_IRQHandler
#define K230_INST_INT_IRQN                                        UART1_INT_IRQn
#define GPIO_K230_RX_PORT                                                  GPIOA
#define GPIO_K230_TX_PORT                                                  GPIOA
#define GPIO_K230_RX_PIN                                           DL_GPIO_PIN_9
#define GPIO_K230_TX_PIN                                           DL_GPIO_PIN_8
#define GPIO_K230_IOMUX_RX                                       (IOMUX_PINCM20)
#define GPIO_K230_IOMUX_TX                                       (IOMUX_PINCM19)
#define GPIO_K230_IOMUX_RX_FUNC                        IOMUX_PINCM20_PF_UART1_RX
#define GPIO_K230_IOMUX_TX_FUNC                        IOMUX_PINCM19_PF_UART1_TX
#define K230_BAUD_RATE                                                    (9600)
#define K230_IBRD_40_MHZ_9600_BAUD                                         (260)
#define K230_FBRD_40_MHZ_9600_BAUD                                          (27)
/* Defines for x_bujin */
#define x_bujin_INST                                                       UART2
#define x_bujin_INST_FREQUENCY                                          40000000
#define x_bujin_INST_IRQHandler                                 UART2_IRQHandler
#define x_bujin_INST_INT_IRQN                                     UART2_INT_IRQn
#define GPIO_x_bujin_RX_PORT                                               GPIOB
#define GPIO_x_bujin_TX_PORT                                               GPIOB
#define GPIO_x_bujin_RX_PIN                                       DL_GPIO_PIN_16
#define GPIO_x_bujin_TX_PIN                                       DL_GPIO_PIN_15
#define GPIO_x_bujin_IOMUX_RX                                    (IOMUX_PINCM33)
#define GPIO_x_bujin_IOMUX_TX                                    (IOMUX_PINCM32)
#define GPIO_x_bujin_IOMUX_RX_FUNC                     IOMUX_PINCM33_PF_UART2_RX
#define GPIO_x_bujin_IOMUX_TX_FUNC                     IOMUX_PINCM32_PF_UART2_TX
#define x_bujin_BAUD_RATE                                               (115200)
#define x_bujin_IBRD_40_MHZ_115200_BAUD                                     (21)
#define x_bujin_FBRD_40_MHZ_115200_BAUD                                     (45)
/* Defines for y_bujin */
#define y_bujin_INST                                                       UART3
#define y_bujin_INST_FREQUENCY                                          80000000
#define y_bujin_INST_IRQHandler                                 UART3_IRQHandler
#define y_bujin_INST_INT_IRQN                                     UART3_INT_IRQn
#define GPIO_y_bujin_RX_PORT                                               GPIOB
#define GPIO_y_bujin_TX_PORT                                               GPIOB
#define GPIO_y_bujin_RX_PIN                                        DL_GPIO_PIN_3
#define GPIO_y_bujin_TX_PIN                                        DL_GPIO_PIN_2
#define GPIO_y_bujin_IOMUX_RX                                    (IOMUX_PINCM16)
#define GPIO_y_bujin_IOMUX_TX                                    (IOMUX_PINCM15)
#define GPIO_y_bujin_IOMUX_RX_FUNC                     IOMUX_PINCM16_PF_UART3_RX
#define GPIO_y_bujin_IOMUX_TX_FUNC                     IOMUX_PINCM15_PF_UART3_TX
#define y_bujin_BAUD_RATE                                                 (9600)
#define y_bujin_IBRD_80_MHZ_9600_BAUD                                      (520)
#define y_bujin_FBRD_80_MHZ_9600_BAUD                                       (53)




/* Defines for SPI_LCD */
#define SPI_LCD_INST                                                       SPI1
#define SPI_LCD_INST_IRQHandler                                 SPI1_IRQHandler
#define SPI_LCD_INST_INT_IRQN                                     SPI1_INT_IRQn
#define GPIO_SPI_LCD_PICO_PORT                                            GPIOB
#define GPIO_SPI_LCD_PICO_PIN                                     DL_GPIO_PIN_8
#define GPIO_SPI_LCD_IOMUX_PICO                                 (IOMUX_PINCM25)
#define GPIO_SPI_LCD_IOMUX_PICO_FUNC                 IOMUX_PINCM25_PF_SPI1_PICO
#define GPIO_SPI_LCD_POCI_PORT                                            GPIOA
#define GPIO_SPI_LCD_POCI_PIN                                    DL_GPIO_PIN_16
#define GPIO_SPI_LCD_IOMUX_POCI                                 (IOMUX_PINCM38)
#define GPIO_SPI_LCD_IOMUX_POCI_FUNC                 IOMUX_PINCM38_PF_SPI1_POCI
/* GPIO configuration for SPI_LCD */
#define GPIO_SPI_LCD_SCLK_PORT                                            GPIOB
#define GPIO_SPI_LCD_SCLK_PIN                                     DL_GPIO_PIN_9
#define GPIO_SPI_LCD_IOMUX_SCLK                                 (IOMUX_PINCM26)
#define GPIO_SPI_LCD_IOMUX_SCLK_FUNC                 IOMUX_PINCM26_PF_SPI1_SCLK



/* Defines for ADC_A15 */
#define ADC_A15_INST                                                        ADC1
#define ADC_A15_INST_IRQHandler                                  ADC1_IRQHandler
#define ADC_A15_INST_INT_IRQN                                    (ADC1_INT_IRQn)
#define ADC_A15_ADCMEM_0                                      DL_ADC12_MEM_IDX_0
#define ADC_A15_ADCMEM_0_REF                     DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_A15_ADCMEM_0_REF_VOLTAGE_V                                       3.3
#define GPIO_ADC_A15_C0_PORT                                               GPIOA
#define GPIO_ADC_A15_C0_PIN                                       DL_GPIO_PIN_15
#define GPIO_ADC_A15_IOMUX_C0                                    (IOMUX_PINCM37)
#define GPIO_ADC_A15_IOMUX_C0_FUNC                (IOMUX_PINCM37_PF_UNCONNECTED)

/* Defines for huidu */
#define huidu_INST                                                          ADC0
#define huidu_INST_IRQHandler                                    ADC0_IRQHandler
#define huidu_INST_INT_IRQN                                      (ADC0_INT_IRQn)
#define huidu_ADCMEM_1_A27                                    DL_ADC12_MEM_IDX_0
#define huidu_ADCMEM_1_A27_REF                   DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define huidu_ADCMEM_1_A27_REF_VOLTAGE_V                                     3.3
#define GPIO_huidu_C0_PORT                                                 GPIOA
#define GPIO_huidu_C0_PIN                                         DL_GPIO_PIN_27
#define GPIO_huidu_IOMUX_C0                                      (IOMUX_PINCM60)
#define GPIO_huidu_IOMUX_C0_FUNC                  (IOMUX_PINCM60_PF_UNCONNECTED)



/* Defines for DMA_CH0 */
#define DMA_CH0_CHAN_ID                                                      (0)
#define huidu_INST_DMA_TRIGGER                        (DMA_ADC0_EVT_GEN_BD_TRIG)


/* Port definition for Pin Group BEEP */
#define BEEP_PORT                                                        (GPIOA)

/* Defines for A29: GPIOA.12 with pinCMx 34 on package pin 5 */
#define BEEP_A29_PIN                                            (DL_GPIO_PIN_12)
#define BEEP_A29_IOMUX                                           (IOMUX_PINCM34)
/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOA)

/* Defines for B22: GPIOA.14 with pinCMx 36 on package pin 7 */
#define LED_B22_PIN                                             (DL_GPIO_PIN_14)
#define LED_B22_IOMUX                                            (IOMUX_PINCM36)
/* Port definition for Pin Group LCD */
#define LCD_PORT                                                         (GPIOB)

/* Defines for RES: GPIOB.10 with pinCMx 27 on package pin 62 */
#define LCD_RES_PIN                                             (DL_GPIO_PIN_10)
#define LCD_RES_IOMUX                                            (IOMUX_PINCM27)
/* Defines for DC: GPIOB.11 with pinCMx 28 on package pin 63 */
#define LCD_DC_PIN                                              (DL_GPIO_PIN_11)
#define LCD_DC_IOMUX                                             (IOMUX_PINCM28)
/* Defines for CS: GPIOB.14 with pinCMx 31 on package pin 2 */
#define LCD_CS_PIN                                              (DL_GPIO_PIN_14)
#define LCD_CS_IOMUX                                             (IOMUX_PINCM31)
/* Defines for BLK: GPIOB.26 with pinCMx 57 on package pin 28 */
#define LCD_BLK_PIN                                             (DL_GPIO_PIN_26)
#define LCD_BLK_IOMUX                                            (IOMUX_PINCM57)
/* Port definition for Pin Group bianma1 */
#define bianma1_PORT                                                     (GPIOA)

/* Defines for read_A28: GPIOA.28 with pinCMx 3 on package pin 35 */
#define bianma1_read_A28_PIN                                    (DL_GPIO_PIN_28)
#define bianma1_read_A28_IOMUX                                    (IOMUX_PINCM3)
/* Defines for read_A31: GPIOA.31 with pinCMx 6 on package pin 39 */
#define bianma1_read_A31_PIN                                    (DL_GPIO_PIN_31)
#define bianma1_read_A31_IOMUX                                    (IOMUX_PINCM6)
/* Port definition for Pin Group bianma2 */
#define bianma2_PORT                                                     (GPIOB)

/* Defines for read_B04: GPIOB.4 with pinCMx 17 on package pin 52 */
#define bianma2_read_B04_PIN                                     (DL_GPIO_PIN_4)
#define bianma2_read_B04_IOMUX                                   (IOMUX_PINCM17)
/* Defines for read_B05: GPIOB.5 with pinCMx 18 on package pin 53 */
#define bianma2_read_B05_PIN                                     (DL_GPIO_PIN_5)
#define bianma2_read_B05_IOMUX                                   (IOMUX_PINCM18)
/* Defines for AIN1_B06: GPIOB.19 with pinCMx 45 on package pin 16 */
#define TB6612_AIN1_B06_PORT                                             (GPIOB)
#define TB6612_AIN1_B06_PIN                                     (DL_GPIO_PIN_19)
#define TB6612_AIN1_B06_IOMUX                                    (IOMUX_PINCM45)
/* Defines for AIN2_B07: GPIOB.7 with pinCMx 24 on package pin 59 */
#define TB6612_AIN2_B07_PORT                                             (GPIOB)
#define TB6612_AIN2_B07_PIN                                      (DL_GPIO_PIN_7)
#define TB6612_AIN2_B07_IOMUX                                    (IOMUX_PINCM24)
/* Defines for BIN1_B23: GPIOA.21 with pinCMx 46 on package pin 17 */
#define TB6612_BIN1_B23_PORT                                             (GPIOA)
#define TB6612_BIN1_B23_PIN                                     (DL_GPIO_PIN_21)
#define TB6612_BIN1_B23_IOMUX                                    (IOMUX_PINCM46)
/* Defines for BIN2_B27: GPIOA.22 with pinCMx 47 on package pin 18 */
#define TB6612_BIN2_B27_PORT                                             (GPIOA)
#define TB6612_BIN2_B27_PIN                                     (DL_GPIO_PIN_22)
#define TB6612_BIN2_B27_IOMUX                                    (IOMUX_PINCM47)
/* Port definition for Pin Group chaosheng */
#define chaosheng_PORT                                                   (GPIOB)

/* Defines for Trig_B17: GPIOB.18 with pinCMx 44 on package pin 15 */
#define chaosheng_Trig_B17_PIN                                  (DL_GPIO_PIN_18)
#define chaosheng_Trig_B17_IOMUX                                 (IOMUX_PINCM44)
/* Defines for Echo_B18: GPIOB.17 with pinCMx 43 on package pin 14 */
#define chaosheng_Echo_B18_PIN                                  (DL_GPIO_PIN_17)
#define chaosheng_Echo_B18_IOMUX                                 (IOMUX_PINCM43)
/* Defines for B25_2: GPIOB.20 with pinCMx 48 on package pin 19 */
#define ghuidu_B25_2_PORT                                                (GPIOB)
#define ghuidu_B25_2_PIN                                        (DL_GPIO_PIN_20)
#define ghuidu_B25_2_IOMUX                                       (IOMUX_PINCM48)
/* Defines for A24_4: GPIOB.21 with pinCMx 49 on package pin 20 */
#define ghuidu_A24_4_PORT                                                (GPIOB)
#define ghuidu_A24_4_PIN                                        (DL_GPIO_PIN_21)
#define ghuidu_A24_4_IOMUX                                       (IOMUX_PINCM49)
/* Defines for B19_3: GPIOB.22 with pinCMx 50 on package pin 21 */
#define ghuidu_B19_3_PORT                                                (GPIOB)
#define ghuidu_B19_3_PIN                                        (DL_GPIO_PIN_22)
#define ghuidu_B19_3_IOMUX                                       (IOMUX_PINCM50)
/* Defines for A14_6: GPIOB.23 with pinCMx 51 on package pin 22 */
#define ghuidu_A14_6_PORT                                                (GPIOB)
#define ghuidu_A14_6_PIN                                        (DL_GPIO_PIN_23)
#define ghuidu_A14_6_IOMUX                                       (IOMUX_PINCM51)
/* Defines for A27_1: GPIOB.24 with pinCMx 52 on package pin 23 */
#define ghuidu_A27_1_PORT                                                (GPIOB)
#define ghuidu_A27_1_PIN                                        (DL_GPIO_PIN_24)
#define ghuidu_A27_1_IOMUX                                       (IOMUX_PINCM52)
/* Defines for A26_5: GPIOA.23 with pinCMx 53 on package pin 24 */
#define ghuidu_A26_5_PORT                                                (GPIOA)
#define ghuidu_A26_5_PIN                                        (DL_GPIO_PIN_23)
#define ghuidu_A26_5_IOMUX                                       (IOMUX_PINCM53)
/* Defines for A07_7: GPIOA.24 with pinCMx 54 on package pin 25 */
#define ghuidu_A07_7_PORT                                                (GPIOA)
#define ghuidu_A07_7_PIN                                        (DL_GPIO_PIN_24)
#define ghuidu_A07_7_IOMUX                                       (IOMUX_PINCM54)
/* Defines for B12_8: GPIOB.25 with pinCMx 56 on package pin 27 */
#define ghuidu_B12_8_PORT                                                (GPIOB)
#define ghuidu_B12_8_PIN                                        (DL_GPIO_PIN_25)
#define ghuidu_B12_8_IOMUX                                       (IOMUX_PINCM56)
/* Defines for K1_B00: GPIOB.27 with pinCMx 58 on package pin 29 */
#define key_K1_B00_PORT                                                  (GPIOB)
#define key_K1_B00_PIN                                          (DL_GPIO_PIN_27)
#define key_K1_B00_IOMUX                                         (IOMUX_PINCM58)
/* Defines for K4_B24: GPIOA.26 with pinCMx 59 on package pin 30 */
#define key_K4_B24_PORT                                                  (GPIOA)
#define key_K4_B24_PIN                                          (DL_GPIO_PIN_26)
#define key_K4_B24_IOMUX                                         (IOMUX_PINCM59)
/* Defines for A22_K3: GPIOA.29 with pinCMx 4 on package pin 36 */
#define key_A22_K3_PORT                                                  (GPIOA)
#define key_A22_K3_PIN                                          (DL_GPIO_PIN_29)
#define key_A22_K3_IOMUX                                          (IOMUX_PINCM4)
/* Defines for K2B_01: GPIOB.1 with pinCMx 13 on package pin 48 */
#define key_K2B_01_PORT                                                  (GPIOB)
#define key_K2B_01_PIN                                           (DL_GPIO_PIN_1)
#define key_K2B_01_IOMUX                                         (IOMUX_PINCM13)
/* Defines for BSL_A18: GPIOA.18 with pinCMx 40 on package pin 11 */
#define key_BSL_A18_PORT                                                 (GPIOA)
#define key_BSL_A18_PIN                                         (DL_GPIO_PIN_18)
#define key_BSL_A18_IOMUX                                        (IOMUX_PINCM40)
/* Defines for USE_key_B21: GPIOA.2 with pinCMx 7 on package pin 42 */
#define key_USE_key_B21_PORT                                             (GPIOA)
#define key_USE_key_B21_PIN                                      (DL_GPIO_PIN_2)
#define key_USE_key_B21_IOMUX                                     (IOMUX_PINCM7)
/* Port definition for Pin Group can */
#define can_PORT                                                         (GPIOA)

/* Defines for TX_A12: GPIOA.3 with pinCMx 8 on package pin 43 */
#define can_TX_A12_PIN                                           (DL_GPIO_PIN_3)
#define can_TX_A12_IOMUX                                          (IOMUX_PINCM8)
/* Defines for RX_A13: GPIOA.13 with pinCMx 35 on package pin 6 */
#define can_RX_A13_PIN                                          (DL_GPIO_PIN_13)
#define can_RX_A13_IOMUX                                         (IOMUX_PINCM35)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_TB6612_PWM_init(void);
void SYSCFG_DL_BIANMA1_TIM_init(void);
void SYSCFG_DL_BIANMA2_TIM_init(void);
void SYSCFG_DL_SYSTEM_TIIM_init(void);
void SYSCFG_DL_MPU9250_init(void);
void SYSCFG_DL_I2C_1_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_K230_init(void);
void SYSCFG_DL_x_bujin_init(void);
void SYSCFG_DL_y_bujin_init(void);
void SYSCFG_DL_SPI_LCD_init(void);
void SYSCFG_DL_ADC_A15_init(void);
void SYSCFG_DL_huidu_init(void);
void SYSCFG_DL_DMA_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
