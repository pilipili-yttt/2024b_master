/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G150X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_SPI_backupConfig gSPI_0Backup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void) {
  SYSCFG_DL_initPower();
  SYSCFG_DL_GPIO_init();
  /* Module-Specific Initializations*/
  SYSCFG_DL_SYSCTL_init();
  SYSCFG_DL_TIMER_0_init();
  SYSCFG_DL_SPI_0_init();
  SYSCFG_DL_ADC12_0_init();
    SYSCFG_DL_VREF_init();
  SYSCFG_DL_OPA_0_init();
  SYSCFG_DL_DMA_init();
  /* Ensure backup structures have no valid state */

  gSPI_0Backup.backupRdy = false;
}
/*
 * User should take care to save and restore register configuration in
 * application. See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void) {
  bool retStatus = true;

  retStatus &= DL_SPI_saveConfiguration(SPI_0_INST, &gSPI_0Backup);

  return retStatus;
}

SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void) {
  bool retStatus = true;

  retStatus &= DL_SPI_restoreConfiguration(SPI_0_INST, &gSPI_0Backup);

  return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void) {
  DL_GPIO_reset(GPIOA);
  DL_GPIO_reset(GPIOB);
  DL_TimerG_reset(TIMER_0_INST);
  DL_SPI_reset(SPI_0_INST);
  DL_ADC12_reset(ADC12_0_INST);
    DL_VREF_reset(VREF);
  DL_OPA_reset(OPA_0_INST);

  DL_GPIO_enablePower(GPIOA);
  DL_GPIO_enablePower(GPIOB);
  DL_TimerG_enablePower(TIMER_0_INST);
  DL_SPI_enablePower(SPI_0_INST);
  DL_ADC12_enablePower(ADC12_0_INST);
    DL_VREF_enablePower(VREF);
  DL_OPA_enablePower(OPA_0_INST);

  delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void) {

  DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXIN_IOMUX);
  DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXOUT_IOMUX);

  DL_GPIO_initPeripheralOutputFunction(GPIO_SPI_0_IOMUX_SCLK,
                                       GPIO_SPI_0_IOMUX_SCLK_FUNC);
  DL_GPIO_initPeripheralOutputFunction(GPIO_SPI_0_IOMUX_PICO,
                                       GPIO_SPI_0_IOMUX_PICO_FUNC);
  DL_GPIO_initPeripheralOutputFunction(GPIO_SPI_0_IOMUX_CD,
                                       GPIO_SPI_0_IOMUX_CD_FUNC);

  DL_GPIO_initDigitalOutput(GPIO_GRP_0_LCD_RST_IOMUX);

  DL_GPIO_initDigitalOutput(GPIO_GRP_0_LED_B16_IOMUX);

  DL_GPIO_initDigitalOutput(GPIO_GRP_0_LCD_CS_IOMUX);

  DL_GPIO_initDigitalInputFeatures(
      GPIO_GRP_0_KEY_LEFT_IOMUX, DL_GPIO_INVERSION_ENABLE,
      DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
      DL_GPIO_WAKEUP_DISABLE);

  DL_GPIO_initDigitalInputFeatures(
      GPIO_GRP_0_KEY_RIGHT_IOMUX, DL_GPIO_INVERSION_ENABLE,
      DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
      DL_GPIO_WAKEUP_DISABLE);

  DL_GPIO_setUpperPinsInputFilter(GPIOA,
                                  DL_GPIO_PIN_31_INPUT_FILTER_8_CYCLES |
                                      DL_GPIO_PIN_28_INPUT_FILTER_8_CYCLES);
  DL_GPIO_clearPins(GPIOB, GPIO_GRP_0_LCD_RST_PIN | GPIO_GRP_0_LED_B16_PIN);
  DL_GPIO_setPins(GPIOB, GPIO_GRP_0_LCD_CS_PIN);
  DL_GPIO_enableOutput(GPIOB, GPIO_GRP_0_LCD_RST_PIN | GPIO_GRP_0_LED_B16_PIN |
                                  GPIO_GRP_0_LCD_CS_PIN);
}

static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq = DL_SYSCTL_SYSPLL_INPUT_FREQ_16_32_MHZ,
    .rDivClk2x = 4,
    .rDivClk1 = 0,
    .rDivClk0 = 4,
    .enableCLK2x = DL_SYSCTL_SYSPLL_CLK2X_ENABLE,
    .enableCLK1 = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
    .enableCLK0 = DL_SYSCTL_SYSPLL_CLK0_DISABLE,
    .sysPLLMCLK = DL_SYSCTL_SYSPLL_MCLK_CLK2X,
    .sysPLLRef = DL_SYSCTL_SYSPLL_REF_HFCLK,
    .qDiv = 3,
    .pDiv = DL_SYSCTL_SYSPLL_PDIV_1};
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void) {

  // Low Power Mode is configured to be SLEEP0
  DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
  DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);

  DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
  DL_SYSCTL_setHFCLKSourceHFXTParams(DL_SYSCTL_HFXT_RANGE_16_32_MHZ, 0, false);
  DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *)&gSYSPLLConfig);
  DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
  DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (40000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   160000 Hz = 40000000 Hz / (1 * (249 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_0ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 249U,
};

/*
 * Timer load value (where the counter starts from) is calculated as
 * (timerPeriod * timerClockFreq) - 1 TIMER_0_INST_LOAD_VALUE = (0.15625 ms *
 * 160000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMER_0TimerConfig = {
    .period = TIMER_0_INST_LOAD_VALUE,
    .timerMode = DL_TIMER_TIMER_MODE_PERIODIC_UP,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_0_init(void) {

  DL_TimerG_setClockConfig(TIMER_0_INST,
                           (DL_TimerG_ClockConfig *)&gTIMER_0ClockConfig);

  DL_TimerG_initTimerMode(TIMER_0_INST,
                          (DL_TimerG_TimerConfig *)&gTIMER_0TimerConfig);
  DL_TimerG_enableClock(TIMER_0_INST);

  DL_TimerG_enableEvent(TIMER_0_INST, DL_TIMERG_EVENT_ROUTE_1,
                        (DL_TIMERG_EVENT_ZERO_EVENT));

  DL_TimerG_setPublisherChanID(TIMER_0_INST, DL_TIMERG_PUBLISHER_INDEX_0,
                               TIMER_0_INST_PUB_0_CH);
}

static const DL_SPI_Config gSPI_0_config = {
    .mode = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0,
    .parity = DL_SPI_PARITY_NONE,
    .dataSize = DL_SPI_DATA_SIZE_8,
    .bitOrder = DL_SPI_BIT_ORDER_MSB_FIRST,
};

static const DL_SPI_ClockConfig gSPI_0_clockConfig = {
    .clockSel = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1};

SYSCONFIG_WEAK void SYSCFG_DL_SPI_0_init(void) {
  DL_SPI_setClockConfig(SPI_0_INST, (DL_SPI_ClockConfig *)&gSPI_0_clockConfig);

  DL_SPI_init(SPI_0_INST, (DL_SPI_Config *)&gSPI_0_config);

  /* Configure Controller mode */
  /*
   * Set the bit rate clock divider to generate the serial output clock
   *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
   *     5000000 = (40000000)/((1 + 3) * 2)
   */
  DL_SPI_setBitRateSerialClockDivider(SPI_0_INST, 3);
  /* Enable and configure CD Mode */
  DL_SPI_enableControllerCommandDataMode(SPI_0_INST);
  DL_SPI_setControllerCommandDataModeConfig(SPI_0_INST, DL_SPI_CD_MODE_COMMAND);
  /* Set RX and TX FIFO threshold levels */
  DL_SPI_setFIFOThreshold(SPI_0_INST, DL_SPI_RX_FIFO_LEVEL_1_2_FULL,
                          DL_SPI_TX_FIFO_LEVEL_1_2_EMPTY);
  //  DL_SPI_enableInterrupt(SPI_0_INST, (DL_SPI_INTERRUPT_IDLE |
  //  DL_SPI_INTERRUPT_TX));

  /* Enable module */
  DL_SPI_enable(SPI_0_INST);
}

/* ADC12_0 Initialization */
static const DL_ADC12_ClockConfig gADC12_0ClockConfig = {
    .clockSel = DL_ADC12_CLOCK_HFCLK,
    .divideRatio = DL_ADC12_CLOCK_DIVIDE_1,
    .freqRange = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC12_0_init(void) {
  DL_ADC12_setClockConfig(ADC12_0_INST,
                          (DL_ADC12_ClockConfig *)&gADC12_0ClockConfig);

  DL_ADC12_initSeqSample(ADC12_0_INST, DL_ADC12_REPEAT_MODE_ENABLED,
                         DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_EVENT,
                         DL_ADC12_SEQ_START_ADDR_00, DL_ADC12_SEQ_END_ADDR_02,
                         DL_ADC12_SAMP_CONV_RES_12_BIT,
                         DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC12_0_INST, ADC12_0_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_8, DL_ADC12_REFERENCE_VOLTAGE_INTREF, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_ENABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(ADC12_0_INST, ADC12_0_ADCMEM_1,
        DL_ADC12_INPUT_CHAN_4, DL_ADC12_REFERENCE_VOLTAGE_INTREF, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_ENABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(ADC12_0_INST, ADC12_0_ADCMEM_2,
        DL_ADC12_INPUT_CHAN_13, DL_ADC12_REFERENCE_VOLTAGE_INTREF, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_ENABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_TRIGGER_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
  DL_ADC12_enableFIFO(ADC12_0_INST);
  DL_ADC12_configHwAverage(ADC12_0_INST, DL_ADC12_HW_AVG_NUM_ACC_8,
                           DL_ADC12_HW_AVG_DEN_DIV_BY_1);

  DL_ADC12_setSampleTime0(ADC12_0_INST, 20);
  DL_ADC12_enableDMA(ADC12_0_INST);
  DL_ADC12_setDMASamplesCnt(ADC12_0_INST, 6);
  DL_ADC12_enableDMATrigger(ADC12_0_INST, (DL_ADC12_DMA_MEM10_RESULT_LOADED));
  DL_ADC12_setSubscriberChanID(ADC12_0_INST, ADC12_0_INST_SUB_CH);
  /* Enable ADC12 interrupt */
  DL_ADC12_clearInterruptStatus(ADC12_0_INST, (DL_ADC12_INTERRUPT_DMA_DONE));
  DL_ADC12_enableInterrupt(ADC12_0_INST, (DL_ADC12_INTERRUPT_DMA_DONE));
  DL_ADC12_enableConversions(ADC12_0_INST);
}


static const DL_VREF_Config gVREFConfig = {
    .vrefEnable     = DL_VREF_ENABLE_ENABLE,
    .bufConfig      = DL_VREF_BUFCONFIG_OUTPUT_2_5V,
    .shModeEnable   = DL_VREF_SHMODE_DISABLE,
    .holdCycleCount = DL_VREF_HOLD_MIN,
    .shCycleCount   = DL_VREF_SH_MIN,
};

SYSCONFIG_WEAK void SYSCFG_DL_VREF_init(void) {
    DL_VREF_configReference(VREF,
        (DL_VREF_Config *) &gVREFConfig);
}


static const DL_OPA_Config gOPA_0Config0 = {
    .pselChannel = DL_OPA_PSEL_IN0_POS,
    .nselChannel = DL_OPA_NSEL_RTAP,
    .mselChannel = DL_OPA_MSEL_IN1_NEG,
    .gain = DL_OPA_GAIN_N1_P2,
    .outputPinState = DL_OPA_OUTPUT_PIN_ENABLED,
    .choppingMode = DL_OPA_CHOPPING_MODE_DISABLE,
};

SYSCONFIG_WEAK void SYSCFG_DL_OPA_0_init(void) {
  DL_OPA_init(OPA_0_INST, (DL_OPA_Config *)&gOPA_0Config0);
  DL_OPA_enableRailToRailInput(OPA_0_INST);

  DL_OPA_enable(OPA_0_INST);
}

static const DL_DMA_Config gDMA_CH0Config = {
    .transferMode = DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE,
    .extendedMode = DL_DMA_NORMAL_MODE,
    .destIncrement = DL_DMA_ADDR_INCREMENT,
    .srcIncrement = DL_DMA_ADDR_UNCHANGED,
    .destWidth = DL_DMA_WIDTH_WORD,
    .srcWidth = DL_DMA_WIDTH_WORD,
    .trigger = ADC12_0_INST_DMA_TRIGGER,
    .triggerType = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH0_init(void) {
  DL_DMA_initChannel(DMA, DMA_CH0_CHAN_ID, (DL_DMA_Config *)&gDMA_CH0Config);
}
SYSCONFIG_WEAK void SYSCFG_DL_DMA_init(void) { SYSCFG_DL_DMA_CH0_init(); }
