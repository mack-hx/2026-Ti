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
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerG_backupConfig gBIANMA1_TIMBackup;
DL_TimerA_backupConfig gBIANMA2_TIMBackup;
DL_TimerA_backupConfig gSYSTEM_TIIMBackup;
DL_UART_Main_backupConfig gy_bujinBackup;
DL_SPI_backupConfig gSPI_LCDBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_TB6612_PWM_init();
    SYSCFG_DL_BIANMA1_TIM_init();
    SYSCFG_DL_BIANMA2_TIM_init();
    SYSCFG_DL_SYSTEM_TIIM_init();
    SYSCFG_DL_MPU9250_init();
    SYSCFG_DL_I2C_1_init();
    SYSCFG_DL_UART_0_init();
    SYSCFG_DL_K230_init();
    SYSCFG_DL_x_bujin_init();
    SYSCFG_DL_y_bujin_init();
    SYSCFG_DL_SPI_LCD_init();
    SYSCFG_DL_ADC_A15_init();
    SYSCFG_DL_huidu_init();
    SYSCFG_DL_DMA_init();
    SYSCFG_DL_SYSTICK_init();
    /* Ensure backup structures have no valid state */

	gBIANMA1_TIMBackup.backupRdy 	= false;
	gBIANMA2_TIMBackup.backupRdy 	= false;
	gSYSTEM_TIIMBackup.backupRdy 	= false;
	gy_bujinBackup.backupRdy 	= false;
	gSPI_LCDBackup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_saveConfiguration(BIANMA1_TIM_INST, &gBIANMA1_TIMBackup);
	retStatus &= DL_TimerA_saveConfiguration(BIANMA2_TIM_INST, &gBIANMA2_TIMBackup);
	retStatus &= DL_TimerA_saveConfiguration(SYSTEM_TIIM_INST, &gSYSTEM_TIIMBackup);
	retStatus &= DL_UART_Main_saveConfiguration(y_bujin_INST, &gy_bujinBackup);
	retStatus &= DL_SPI_saveConfiguration(SPI_LCD_INST, &gSPI_LCDBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_restoreConfiguration(BIANMA1_TIM_INST, &gBIANMA1_TIMBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(BIANMA2_TIM_INST, &gBIANMA2_TIMBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(SYSTEM_TIIM_INST, &gSYSTEM_TIIMBackup, false);
	retStatus &= DL_UART_Main_restoreConfiguration(y_bujin_INST, &gy_bujinBackup);
	retStatus &= DL_SPI_restoreConfiguration(SPI_LCD_INST, &gSPI_LCDBackup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerG_reset(TB6612_PWM_INST);
    DL_TimerG_reset(BIANMA1_TIM_INST);
    DL_TimerA_reset(BIANMA2_TIM_INST);
    DL_TimerA_reset(SYSTEM_TIIM_INST);
    DL_I2C_reset(MPU9250_INST);
    DL_I2C_reset(I2C_1_INST);
    DL_UART_Main_reset(UART_0_INST);
    DL_UART_Main_reset(K230_INST);
    DL_UART_Main_reset(x_bujin_INST);
    DL_UART_Main_reset(y_bujin_INST);
    DL_SPI_reset(SPI_LCD_INST);
    DL_ADC12_reset(ADC_A15_INST);
    DL_ADC12_reset(huidu_INST);



    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerG_enablePower(TB6612_PWM_INST);
    DL_TimerG_enablePower(BIANMA1_TIM_INST);
    DL_TimerA_enablePower(BIANMA2_TIM_INST);
    DL_TimerA_enablePower(SYSTEM_TIIM_INST);
    DL_I2C_enablePower(MPU9250_INST);
    DL_I2C_enablePower(I2C_1_INST);
    DL_UART_Main_enablePower(UART_0_INST);
    DL_UART_Main_enablePower(K230_INST);
    DL_UART_Main_enablePower(x_bujin_INST);
    DL_UART_Main_enablePower(y_bujin_INST);
    DL_SPI_enablePower(SPI_LCD_INST);
    DL_ADC12_enablePower(ADC_A15_INST);
    DL_ADC12_enablePower(huidu_INST);


    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_TB6612_PWM_C0_IOMUX,GPIO_TB6612_PWM_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_TB6612_PWM_C0_PORT, GPIO_TB6612_PWM_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_TB6612_PWM_C1_IOMUX,GPIO_TB6612_PWM_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_TB6612_PWM_C1_PORT, GPIO_TB6612_PWM_C1_PIN);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_MPU9250_IOMUX_SDA,
        GPIO_MPU9250_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_MPU9250_IOMUX_SCL,
        GPIO_MPU9250_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_MPU9250_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_MPU9250_IOMUX_SCL);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_1_IOMUX_SDA,
        GPIO_I2C_1_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_1_IOMUX_SCL,
        GPIO_I2C_1_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_1_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_1_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_0_IOMUX_TX, GPIO_UART_0_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_0_IOMUX_RX, GPIO_UART_0_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_K230_IOMUX_TX, GPIO_K230_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_K230_IOMUX_RX, GPIO_K230_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_x_bujin_IOMUX_TX, GPIO_x_bujin_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_x_bujin_IOMUX_RX, GPIO_x_bujin_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_y_bujin_IOMUX_TX, GPIO_y_bujin_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_y_bujin_IOMUX_RX, GPIO_y_bujin_IOMUX_RX_FUNC);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_LCD_IOMUX_SCLK, GPIO_SPI_LCD_IOMUX_SCLK_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_LCD_IOMUX_PICO, GPIO_SPI_LCD_IOMUX_PICO_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_SPI_LCD_IOMUX_POCI, GPIO_SPI_LCD_IOMUX_POCI_FUNC);

    DL_GPIO_initDigitalOutput(BEEP_A29_IOMUX);

    DL_GPIO_initDigitalOutput(LED_B22_IOMUX);

    DL_GPIO_initDigitalOutput(LCD_RES_IOMUX);

    DL_GPIO_initDigitalOutput(LCD_DC_IOMUX);

    DL_GPIO_initDigitalOutput(LCD_CS_IOMUX);

    DL_GPIO_initDigitalOutput(LCD_BLK_IOMUX);

    DL_GPIO_initDigitalInputFeatures(bianma1_read_A28_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(bianma1_read_A31_IOMUX);

    DL_GPIO_initDigitalInputFeatures(bianma2_read_B04_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(bianma2_read_B05_IOMUX);

    DL_GPIO_initDigitalOutput(TB6612_AIN1_B06_IOMUX);

    DL_GPIO_initDigitalOutput(TB6612_AIN2_B07_IOMUX);

    DL_GPIO_initDigitalOutput(TB6612_BIN1_B23_IOMUX);

    DL_GPIO_initDigitalOutput(TB6612_BIN2_B27_IOMUX);

    DL_GPIO_initDigitalOutput(chaosheng_Trig_B17_IOMUX);

    DL_GPIO_initDigitalInputFeatures(chaosheng_Echo_B18_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ghuidu_B25_2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ghuidu_A24_4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ghuidu_B19_3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ghuidu_A14_6_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ghuidu_A27_1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ghuidu_A26_5_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ghuidu_A07_7_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ghuidu_B12_8_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(key_K1_B00_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(key_K4_B24_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(key_K3_A22_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(key_K2B_01_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(key_USE_key_B21_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(can_TX_A12_IOMUX);

    DL_GPIO_initDigitalInputFeatures(can_RX_A13_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOA, BEEP_A29_PIN |
		bianma1_read_A31_PIN |
		TB6612_AIN1_B06_PIN |
		TB6612_BIN1_B23_PIN |
		can_TX_A12_PIN);
    DL_GPIO_enableOutput(GPIOA, BEEP_A29_PIN |
		bianma1_read_A31_PIN |
		TB6612_AIN1_B06_PIN |
		TB6612_BIN1_B23_PIN |
		can_TX_A12_PIN);
    DL_GPIO_clearPins(GPIOB, LED_B22_PIN |
		LCD_RES_PIN |
		LCD_DC_PIN |
		LCD_CS_PIN |
		LCD_BLK_PIN |
		bianma2_read_B05_PIN |
		TB6612_AIN2_B07_PIN |
		TB6612_BIN2_B27_PIN |
		chaosheng_Trig_B17_PIN);
    DL_GPIO_enableOutput(GPIOB, LED_B22_PIN |
		LCD_RES_PIN |
		LCD_DC_PIN |
		LCD_CS_PIN |
		LCD_BLK_PIN |
		bianma2_read_B05_PIN |
		TB6612_AIN2_B07_PIN |
		TB6612_BIN2_B27_PIN |
		chaosheng_Trig_B17_PIN);

}


static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq              = DL_SYSCTL_SYSPLL_INPUT_FREQ_16_32_MHZ,
	.rDivClk2x              = 1,
	.rDivClk1               = 0,
	.rDivClk0               = 0,
	.enableCLK2x            = DL_SYSCTL_SYSPLL_CLK2X_ENABLE,
	.enableCLK1             = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
	.enableCLK0             = DL_SYSCTL_SYSPLL_CLK0_DISABLE,
	.sysPLLMCLK             = DL_SYSCTL_SYSPLL_MCLK_CLK2X,
	.sysPLLRef              = DL_SYSCTL_SYSPLL_REF_SYSOSC,
	.qDiv                   = 4,
	.pDiv                   = DL_SYSCTL_SYSPLL_PDIV_2
};

SYSCONFIG_WEAK bool SYSCFG_DL_SYSCTL_SYSPLL_init(void)
{
    bool fFCCRatioStatus = false;
    uint32_t fFCCSysoscCount;
    uint32_t fFCCPllCount;
    uint32_t fFCCRatio;
    uint32_t fccTimeOutCounter;

    DL_SYSCTL_setFCCPeriods( DL_SYSCTL_FCC_TRIG_CNT_01 );

    /* Measuring PLL. */
    DL_SYSCTL_configFCC(DL_SYSCTL_FCC_TRIG_TYPE_RISE_RISE,
                        DL_SYSCTL_FCC_TRIG_SOURCE_LFCLK,
                        DL_SYSCTL_FCC_CLOCK_SOURCE_SYSPLLCLK2X);
    /* Get SYSPLL frequency using FCC */
    fccTimeOutCounter = 0;
    DL_SYSCTL_startFCC();
    while (DL_SYSCTL_isFCCDone() == 0) {
        delay_cycles(977);  /* 1x LFCLK cycle = 32MHz/32.768kHz = 977, 30.5us */
        fccTimeOutCounter++;
        if(fccTimeOutCounter > 65){
            /* Timeout set to approximately 2ms (user-customizable) */
            break;
        }
    }

    /* get measA= SYSPLLCLK2X freq wrt LFOSC*/
    fFCCPllCount = DL_SYSCTL_readFCC();

    /* Measuring SYSPLL Source */
    DL_SYSCTL_configFCC(DL_SYSCTL_FCC_TRIG_TYPE_RISE_RISE,
                        DL_SYSCTL_FCC_TRIG_SOURCE_LFCLK,
                        DL_SYSCTL_FCC_CLOCK_SOURCE_SYSOSC);
    /* Get SYSPLL frequency using FCC */
    fccTimeOutCounter = 0;
    DL_SYSCTL_startFCC();
    while (DL_SYSCTL_isFCCDone() == 0) {
        delay_cycles(977);  /* 1x LFCLK cycle = 32MHz/32.768kHz = 977, 30.5us */
        fccTimeOutCounter++;
        if(fccTimeOutCounter > 65){
            /* Timeout set to approximately 2ms (user-customizable) */
            break;
        }
    }

    /* get measB= SYSOSC freq wrt LFOSC*/
    fFCCSysoscCount = DL_SYSCTL_readFCC();

    /* Get ratio of both measurements*/
    fFCCRatio = (fFCCPllCount * FLOAT_TO_INT_SCALE) / fFCCSysoscCount;
    /* Check ratio is within bounds*/
    if ((FCC_LOWER_BOUND <  fFCCRatio) && (fFCCRatio < FCC_UPPER_BOUND))
    {
        /* ratio is good for proceeding into application code. */
        fFCCRatioStatus = true;
    }

    return fFCCRatioStatus;
}
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	/* Set default configuration */
	DL_SYSCTL_disableHFXT();
	DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);

    /*
     * [SYSPLL_ERR_01]
     * PLL Incorrect locking WA start.
     * Insert after every PLL enable.
     * This can lead an infinite loop if the condition persists
     * and can block entry to the application code.
     */

    while (SYSCFG_DL_SYSCTL_SYSPLL_init() == false)
    {
        /* Toggle SYSPLL enable to re-enable SYSPLL and re-check incorrect locking */
        DL_SYSCTL_disableSYSPLL();
        DL_SYSCTL_enableSYSPLL();

        /* Wait until SYSPLL startup is stabilized*/
        while ((DL_SYSCTL_getClockStatus() & SYSCTL_CLKSTATUS_SYSPLLGOOD_MASK) != DL_SYSCTL_CLK_STATUS_SYSPLL_GOOD){}
    }
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);

}


/*
 * Timer clock configuration to be sourced by  / 2 (40000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   40000000 Hz = 40000000 Hz / (2 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gTB6612_PWMClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_2,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gTB6612_PWMConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 2000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TB6612_PWM_init(void) {

    DL_TimerG_setClockConfig(
        TB6612_PWM_INST, (DL_TimerG_ClockConfig *) &gTB6612_PWMClockConfig);

    DL_TimerG_initPWMMode(
        TB6612_PWM_INST, (DL_TimerG_PWMConfig *) &gTB6612_PWMConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(TB6612_PWM_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(TB6612_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(TB6612_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(TB6612_PWM_INST, 2000, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(TB6612_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(TB6612_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(TB6612_PWM_INST, 2000, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(TB6612_PWM_INST);


    
    DL_TimerG_setCCPDirection(TB6612_PWM_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (10000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   1000000 Hz = 10000000 Hz / (8 * (9 + 1))
 */
static const DL_TimerG_ClockConfig gBIANMA1_TIMClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 9U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * BIANMA1_TIM_INST_LOAD_VALUE = (50 ms * 1000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gBIANMA1_TIMTimerConfig = {
    .period     = BIANMA1_TIM_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC_UP,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_BIANMA1_TIM_init(void) {

    DL_TimerG_setClockConfig(BIANMA1_TIM_INST,
        (DL_TimerG_ClockConfig *) &gBIANMA1_TIMClockConfig);

    DL_TimerG_initTimerMode(BIANMA1_TIM_INST,
        (DL_TimerG_TimerConfig *) &gBIANMA1_TIMTimerConfig);
    DL_TimerG_enableClock(BIANMA1_TIM_INST);





}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (10000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   1000000 Hz = 10000000 Hz / (8 * (9 + 1))
 */
static const DL_TimerA_ClockConfig gBIANMA2_TIMClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 9U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * BIANMA2_TIM_INST_LOAD_VALUE = (50.00 ms * 1000000 Hz) - 1
 */
static const DL_TimerA_TimerConfig gBIANMA2_TIMTimerConfig = {
    .period     = BIANMA2_TIM_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC_UP,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_BIANMA2_TIM_init(void) {

    DL_TimerA_setClockConfig(BIANMA2_TIM_INST,
        (DL_TimerA_ClockConfig *) &gBIANMA2_TIMClockConfig);

    DL_TimerA_initTimerMode(BIANMA2_TIM_INST,
        (DL_TimerA_TimerConfig *) &gBIANMA2_TIMTimerConfig);
    DL_TimerA_enableClock(BIANMA2_TIM_INST);





}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (10000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   10000000 Hz = 10000000 Hz / (8 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gSYSTEM_TIIMClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 0U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * SYSTEM_TIIM_INST_LOAD_VALUE = (1 ms * 10000000 Hz) - 1
 */
static const DL_TimerA_TimerConfig gSYSTEM_TIIMTimerConfig = {
    .period     = SYSTEM_TIIM_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_ONE_SHOT,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_SYSTEM_TIIM_init(void) {

    DL_TimerA_setClockConfig(SYSTEM_TIIM_INST,
        (DL_TimerA_ClockConfig *) &gSYSTEM_TIIMClockConfig);

    DL_TimerA_initTimerMode(SYSTEM_TIIM_INST,
        (DL_TimerA_TimerConfig *) &gSYSTEM_TIIMTimerConfig);
    DL_TimerA_enableClock(SYSTEM_TIIM_INST);





}


static const DL_I2C_ClockConfig gMPU9250ClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_MPU9250_init(void) {

    DL_I2C_setClockConfig(MPU9250_INST,
        (DL_I2C_ClockConfig *) &gMPU9250ClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(MPU9250_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(MPU9250_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(MPU9250_INST);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(MPU9250_INST, 9);
    DL_I2C_setControllerTXFIFOThreshold(MPU9250_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(MPU9250_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(MPU9250_INST);


    /* Enable module */
    DL_I2C_enableController(MPU9250_INST);


}
static const DL_I2C_ClockConfig gI2C_1ClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_1_init(void) {

    DL_I2C_setClockConfig(I2C_1_INST,
        (DL_I2C_ClockConfig *) &gI2C_1ClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(I2C_1_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(I2C_1_INST);




}

static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_0Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_0_init(void)
{
    DL_UART_Main_setClockConfig(UART_0_INST, (DL_UART_Main_ClockConfig *) &gUART_0ClockConfig);

    DL_UART_Main_init(UART_0_INST, (DL_UART_Main_Config *) &gUART_0Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9599.81
     */
    DL_UART_Main_setOversampling(UART_0_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_0_INST, UART_0_IBRD_40_MHZ_9600_BAUD, UART_0_FBRD_40_MHZ_9600_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_0_INST,
                                 DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
                                 DL_UART_MAIN_INTERRUPT_CTS_DONE |
                                 DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
                                 DL_UART_MAIN_INTERRUPT_DMA_DONE_TX |
                                 DL_UART_MAIN_INTERRUPT_EOT_DONE |
                                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR |
                                 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                 DL_UART_MAIN_INTERRUPT_RX |
                                 DL_UART_MAIN_INTERRUPT_RXD_NEG_EDGE |
                                 DL_UART_MAIN_INTERRUPT_RXD_POS_EDGE |
                                 DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
                                 DL_UART_MAIN_INTERRUPT_TX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(UART_0_INST_INT_IRQN, 0);


    DL_UART_Main_enable(UART_0_INST);
}
static const DL_UART_Main_ClockConfig gK230ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gK230Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_K230_init(void)
{
    DL_UART_Main_setClockConfig(K230_INST, (DL_UART_Main_ClockConfig *) &gK230ClockConfig);

    DL_UART_Main_init(K230_INST, (DL_UART_Main_Config *) &gK230Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9599.81
     */
    DL_UART_Main_setOversampling(K230_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(K230_INST, K230_IBRD_40_MHZ_9600_BAUD, K230_FBRD_40_MHZ_9600_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(K230_INST,
                                 DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(K230_INST_INT_IRQN, 2);


    DL_UART_Main_enable(K230_INST);
}
static const DL_UART_Main_ClockConfig gx_bujinClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gx_bujinConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_x_bujin_init(void)
{
    DL_UART_Main_setClockConfig(x_bujin_INST, (DL_UART_Main_ClockConfig *) &gx_bujinClockConfig);

    DL_UART_Main_init(x_bujin_INST, (DL_UART_Main_Config *) &gx_bujinConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(x_bujin_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(x_bujin_INST, x_bujin_IBRD_40_MHZ_115200_BAUD, x_bujin_FBRD_40_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(x_bujin_INST,
                                 DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
                                 DL_UART_MAIN_INTERRUPT_CTS_DONE |
                                 DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
                                 DL_UART_MAIN_INTERRUPT_DMA_DONE_TX |
                                 DL_UART_MAIN_INTERRUPT_EOT_DONE |
                                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR |
                                 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                 DL_UART_MAIN_INTERRUPT_RX |
                                 DL_UART_MAIN_INTERRUPT_RXD_NEG_EDGE |
                                 DL_UART_MAIN_INTERRUPT_RXD_POS_EDGE |
                                 DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
                                 DL_UART_MAIN_INTERRUPT_TX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(x_bujin_INST_INT_IRQN, 0);


    DL_UART_Main_setRXInterruptTimeout(x_bujin_INST, 10);

    DL_UART_Main_enable(x_bujin_INST);
}
static const DL_UART_Main_ClockConfig gy_bujinClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gy_bujinConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_y_bujin_init(void)
{
    DL_UART_Main_setClockConfig(y_bujin_INST, (DL_UART_Main_ClockConfig *) &gy_bujinClockConfig);

    DL_UART_Main_init(y_bujin_INST, (DL_UART_Main_Config *) &gy_bujinConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(y_bujin_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(y_bujin_INST, y_bujin_IBRD_80_MHZ_115200_BAUD, y_bujin_FBRD_80_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(y_bujin_INST,
                                 DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
                                 DL_UART_MAIN_INTERRUPT_CTS_DONE |
                                 DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
                                 DL_UART_MAIN_INTERRUPT_DMA_DONE_TX |
                                 DL_UART_MAIN_INTERRUPT_EOT_DONE |
                                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR |
                                 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                 DL_UART_MAIN_INTERRUPT_RX |
                                 DL_UART_MAIN_INTERRUPT_RXD_NEG_EDGE |
                                 DL_UART_MAIN_INTERRUPT_RXD_POS_EDGE |
                                 DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
                                 DL_UART_MAIN_INTERRUPT_TX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(y_bujin_INST_INT_IRQN, 1);


    DL_UART_Main_setRXInterruptTimeout(y_bujin_INST, 10);

    DL_UART_Main_enable(y_bujin_INST);
}

static const DL_SPI_Config gSPI_LCD_config = {
    .mode        = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = DL_SPI_DATA_SIZE_8,
    .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
};

static const DL_SPI_ClockConfig gSPI_LCD_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

SYSCONFIG_WEAK void SYSCFG_DL_SPI_LCD_init(void) {
    DL_SPI_setClockConfig(SPI_LCD_INST, (DL_SPI_ClockConfig *) &gSPI_LCD_clockConfig);

    DL_SPI_init(SPI_LCD_INST, (DL_SPI_Config *) &gSPI_LCD_config);

    /* Configure Controller mode */
    /*
     * Set the bit rate clock divider to generate the serial output clock
     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
     *     2000000 = (80000000)/((1 + 19) * 2)
     */
    DL_SPI_setBitRateSerialClockDivider(SPI_LCD_INST, 19);
    /* Set RX and TX FIFO threshold levels */
    DL_SPI_setFIFOThreshold(SPI_LCD_INST, DL_SPI_RX_FIFO_LEVEL_ONE_FRAME, DL_SPI_TX_FIFO_LEVEL_3_4_EMPTY);

    /* Enable module */
    DL_SPI_enable(SPI_LCD_INST);
}

/* ADC_A15 Initialization */
static const DL_ADC12_ClockConfig gADC_A15ClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_1,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC_A15_init(void)
{
    DL_ADC12_setClockConfig(ADC_A15_INST, (DL_ADC12_ClockConfig *) &gADC_A15ClockConfig);
    DL_ADC12_configConversionMem(ADC_A15_INST, ADC_A15_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_enableConversions(ADC_A15_INST);
}
/* huidu Initialization */
static const DL_ADC12_ClockConfig ghuiduClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_ULPCLK,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_8,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_32_TO_40,
};
SYSCONFIG_WEAK void SYSCFG_DL_huidu_init(void)
{
    DL_ADC12_setClockConfig(huidu_INST, (DL_ADC12_ClockConfig *) &ghuiduClockConfig);
    DL_ADC12_initSingleSample(huidu_INST,
        DL_ADC12_REPEAT_MODE_ENABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SAMP_CONV_RES_12_BIT, DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(huidu_INST, huidu_ADCMEM_1_A27,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setSampleTime0(huidu_INST,10);
    DL_ADC12_enableDMA(huidu_INST);
    DL_ADC12_setDMASamplesCnt(huidu_INST,1);
    DL_ADC12_enableDMATrigger(huidu_INST,(DL_ADC12_DMA_MEM0_RESULT_LOADED));
    DL_ADC12_enableConversions(huidu_INST);
}

static const DL_DMA_Config gDMA_CH0Config = {
    .transferMode   = DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_UNCHANGED,
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,
    .destWidth      = DL_DMA_WIDTH_HALF_WORD,
    .srcWidth       = DL_DMA_WIDTH_HALF_WORD,
    .trigger        = huidu_INST_DMA_TRIGGER,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH0_init(void)
{
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, 40);
    DL_DMA_initChannel(DMA, DMA_CH0_CHAN_ID , (DL_DMA_Config *) &gDMA_CH0Config);
}
SYSCONFIG_WEAK void SYSCFG_DL_DMA_init(void){
    SYSCFG_DL_DMA_CH0_init();
}


SYSCONFIG_WEAK void SYSCFG_DL_SYSTICK_init(void)
{
    /*
     * Initializes the SysTick period to 400.00 μs,
     * enables the interrupt, and starts the SysTick Timer
     */
    DL_SYSTICK_config(32000);
}

