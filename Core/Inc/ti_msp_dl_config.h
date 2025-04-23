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
*  DO NOT EDIT - This file is generated for the MSPM0G150X
*  by the SysConfig tool.
*/
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G150X

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


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     40000000



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG0)
#define TIMER_0_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                            (24U)
#define TIMER_0_INST_PUB_0_CH                                                (1)



/* Defines for SPI_0 */
#define SPI_0_INST                                                         SPI1
#define SPI_0_INST_IRQHandler                                   SPI1_IRQHandler
#define SPI_0_INST_INT_IRQN                                       SPI1_INT_IRQn
#define GPIO_SPI_0_PICO_PORT                                              GPIOB
#define GPIO_SPI_0_PICO_PIN                                       DL_GPIO_PIN_8
#define GPIO_SPI_0_IOMUX_PICO                                   (IOMUX_PINCM25)
#define GPIO_SPI_0_IOMUX_PICO_FUNC                   IOMUX_PINCM25_PF_SPI1_PICO
/* GPIO configuration for SPI_0 */
#define GPIO_SPI_0_SCLK_PORT                                              GPIOB
#define GPIO_SPI_0_SCLK_PIN                                       DL_GPIO_PIN_9
#define GPIO_SPI_0_IOMUX_SCLK                                   (IOMUX_PINCM26)
#define GPIO_SPI_0_IOMUX_SCLK_FUNC                   IOMUX_PINCM26_PF_SPI1_SCLK
#define GPIO_SPI_0_CS0_PORT                                               GPIOA
#define GPIO_SPI_0_CS0_PIN                                        DL_GPIO_PIN_2
#define GPIO_SPI_0_IOMUX_CS0                                     (IOMUX_PINCM7)
#define GPIO_SPI_0_IOMUX_CS0_FUNC                      IOMUX_PINCM7_PF_SPI1_CS0
#define GPIO_SPI_0_CD_PORT                                                GPIOB
#define GPIO_SPI_0_CD_PIN                                        DL_GPIO_PIN_14
#define GPIO_SPI_0_IOMUX_CD                                     (IOMUX_PINCM31)
#define GPIO_SPI_0_IOMUX_CD_FUNC             IOMUX_PINCM31_PF_SPI1_CS3_CD_POCI3



/* Defines for ADC12_0 */
#define ADC12_0_INST                                                        ADC1
#define ADC12_0_INST_IRQHandler                                  ADC1_IRQHandler
#define ADC12_0_INST_INT_IRQN                                    (ADC1_INT_IRQn)
#define ADC12_0_ADCMEM_0                                      DL_ADC12_MEM_IDX_0
#define ADC12_0_ADCMEM_0_REF                   DL_ADC12_REFERENCE_VOLTAGE_INTREF
#define ADC12_0_ADCMEM_0_REF_VOLTAGE_V                                      2.50
#define ADC12_0_ADCMEM_1                                      DL_ADC12_MEM_IDX_1
#define ADC12_0_ADCMEM_1_REF                   DL_ADC12_REFERENCE_VOLTAGE_INTREF
#define ADC12_0_ADCMEM_1_REF_VOLTAGE_V                                      2.50
#define ADC12_0_ADCMEM_2                                      DL_ADC12_MEM_IDX_2
#define ADC12_0_ADCMEM_2_REF                   DL_ADC12_REFERENCE_VOLTAGE_INTREF
#define ADC12_0_ADCMEM_2_REF_VOLTAGE_V                                      2.50
#define ADC12_0_INST_SUB_CH                                                  (1)
#define GPIO_ADC12_0_C8_PORT                                               GPIOA
#define GPIO_ADC12_0_C8_PIN                                       DL_GPIO_PIN_22
#define GPIO_ADC12_0_C4_PORT                                               GPIOB
#define GPIO_ADC12_0_C4_PIN                                       DL_GPIO_PIN_17
#define GPIO_ADC12_0_C5_PORT                                               GPIOB
#define GPIO_ADC12_0_C5_PIN                                       DL_GPIO_PIN_18


/* Defines for VREF */
#define VREF_VOLTAGE_MV                                                     2500
#define GPIO_VREF_VREFPOS_PORT                                             GPIOA
#define GPIO_VREF_VREFPOS_PIN                                     DL_GPIO_PIN_23
#define GPIO_VREF_IOMUX_VREFPOS                                  (IOMUX_PINCM53)
#define GPIO_VREF_IOMUX_VREFPOS_FUNC                IOMUX_PINCM53_PF_UNCONNECTED



/* Defines for OPA_0 */
#define OPA_0_INST                                                          OPA1
#define GPIO_OPA_0_IN0POS_PORT                                             GPIOB
#define GPIO_OPA_0_IN0POS_PIN                                     DL_GPIO_PIN_19
#define GPIO_OPA_0_IOMUX_IN0POS                                  (IOMUX_PINCM45)
#define GPIO_OPA_0_IOMUX_IN0POS_FUNC                IOMUX_PINCM45_PF_UNCONNECTED
#define GPIO_OPA_0_IN1NEG_PORT                                             GPIOA
#define GPIO_OPA_0_IN1NEG_PIN                                     DL_GPIO_PIN_17
#define GPIO_OPA_0_IOMUX_IN1NEG                                  (IOMUX_PINCM39)
#define GPIO_OPA_0_IOMUX_IN1NEG_FUNC                IOMUX_PINCM39_PF_UNCONNECTED
#define GPIO_OPA_0_OUT_PORT                                                GPIOA
#define GPIO_OPA_0_OUT_PIN                                        DL_GPIO_PIN_16
#define GPIO_OPA_0_IOMUX_OUT                                     (IOMUX_PINCM38)
#define GPIO_OPA_0_IOMUX_OUT_FUNC                   IOMUX_PINCM38_PF_UNCONNECTED



/* Defines for DMA_CH0 */
#define DMA_CH0_CHAN_ID                                                      (0)
#define ADC12_0_INST_DMA_TRIGGER                      (DMA_ADC1_EVT_GEN_BD_TRIG)



/* Defines for LCD_RST: GPIOB.15 with pinCMx 32 on package pin 25 */
#define GPIO_GRP_0_LCD_RST_PORT                                          (GPIOB)
#define GPIO_GRP_0_LCD_RST_PIN                                  (DL_GPIO_PIN_15)
#define GPIO_GRP_0_LCD_RST_IOMUX                                 (IOMUX_PINCM32)
/* Defines for LED_B16: GPIOB.16 with pinCMx 33 on package pin 26 */
#define GPIO_GRP_0_LED_B16_PORT                                          (GPIOB)
#define GPIO_GRP_0_LED_B16_PIN                                  (DL_GPIO_PIN_16)
#define GPIO_GRP_0_LED_B16_IOMUX                                 (IOMUX_PINCM33)
/* Defines for LCD_CS: GPIOB.6 with pinCMx 23 on package pin 20 */
#define GPIO_GRP_0_LCD_CS_PORT                                           (GPIOB)
#define GPIO_GRP_0_LCD_CS_PIN                                    (DL_GPIO_PIN_6)
#define GPIO_GRP_0_LCD_CS_IOMUX                                  (IOMUX_PINCM23)
/* Defines for KEY_LEFT: GPIOA.31 with pinCMx 6 on package pin 5 */
#define GPIO_GRP_0_KEY_LEFT_PORT                                         (GPIOA)
#define GPIO_GRP_0_KEY_LEFT_PIN                                 (DL_GPIO_PIN_31)
#define GPIO_GRP_0_KEY_LEFT_IOMUX                                 (IOMUX_PINCM6)
/* Defines for KEY_RIGHT: GPIOA.28 with pinCMx 3 on package pin 3 */
#define GPIO_GRP_0_KEY_RIGHT_PORT                                        (GPIOA)
#define GPIO_GRP_0_KEY_RIGHT_PIN                                (DL_GPIO_PIN_28)
#define GPIO_GRP_0_KEY_RIGHT_IOMUX                                (IOMUX_PINCM3)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_SPI_0_init(void);
void SYSCFG_DL_ADC12_0_init(void);
void SYSCFG_DL_VREF_init(void);
void SYSCFG_DL_OPA_0_init(void);
void SYSCFG_DL_DMA_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
