/*
 * cmd_adccfg.c
 *
 *  Created on: Nov 25, 2018
 *      Author: kevin
 */

#include <cmd/cmd_adc.h>
#include <stdio.h>
#include "board.h"
#include "fsl_adc.h"
#include "fsl_power.h"

/**
 * This command allows you to configure the ADC, which can be read via
 * the ADC_HI and ADC_LO opcodes.
 *
 * By default, ADC_CHN0 is used via pin P0_7.
 *
 * NOTE: The 'HI' register should always be read before 'LO', since the
 *       ADC conversion is triggered during the 'HI' read.
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CMD_ADC_BASE 					(ADC0)
#define CMD_ADC_SAMPLE_CHANNEL_NUMBER 	(0U)
#define CMD_ADC_CLOCK_SOURCE 			(kCLOCK_Fro)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void cmd_adc_sample(void);
static void cmd_adc_configuration(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
adc_result_info_t l_cmd_adc_result_info;

/* -----------------------------------------------------------------------
 * ADC_HI Command Handlers
 * ----------------------------------------------------------------------- */

static struct cmd_record cmd_adchi_record = {
	.op_code = CMD_OP_CODE_ADC_HI,
	.write = &cmd_adchi_write_cb,
	.read = &cmd_adchi_read_cb,
	.timeout = 0
};

int
cmd_adchi_write_cb(uint8_t op_code, struct cmd_data *pdata, void *arg)
{
	/* This register is read only. */

	return kStatus_Success;
}

int
cmd_adchi_read_cb(uint8_t op_code, struct cmd_data *pdata, void *arg)
{
	if (pdata->len) {
		cmd_adc_sample();
		memset(pdata->payload, 0, pdata->len);
		pdata->payload[0] = (l_cmd_adc_result_info.result >> 8) & 0xFF;
	}

	return kStatus_Success;
}

/* -----------------------------------------------------------------------
 * ADC_LO Command Handlers
 * ----------------------------------------------------------------------- */

static struct cmd_record cmd_adclo_record = {
	.op_code = CMD_OP_CODE_ADC_LO,
	.write = &cmd_adclo_write_cb,
	.read = &cmd_adclo_read_cb,
	.timeout = 0
};

int
cmd_adclo_write_cb(uint8_t op_code, struct cmd_data *pdata, void *arg)
{
	/* This register is read only. */

	return kStatus_Success;
}

int
cmd_adclo_read_cb(uint8_t op_code, struct cmd_data *pdata, void *arg)
{
	if (pdata->len) {
		memset(pdata->payload, 0, pdata->len);
		pdata->payload[0] = l_cmd_adc_result_info.result & 0xFF;
	}

	return kStatus_Success;
}

/* -----------------------------------------------------------------------
 * ADC_STAT Command Handlers
 * ----------------------------------------------------------------------- */

static struct cmd_record cmd_adcstat_record = {
	.op_code = CMD_OP_CODE_ADC_STAT,
	.write = &cmd_adcstat_write_cb,
	.read = &cmd_adcstat_read_cb,
	.timeout = 0
};

int
cmd_adcstat_write_cb(uint8_t op_code, struct cmd_data *pdata, void *arg)
{
	/* This register is read only. */

	return kStatus_Success;
}

int
cmd_adcstat_read_cb(uint8_t op_code, struct cmd_data *pdata, void *arg)
{
	uint8_t stat;

	/* B0   = overrun flag */
	stat =  l_cmd_adc_result_info.overrunFlag ? 1 : 0;
	/* B1:5 = channel (range = 0..7) */
	stat |= l_cmd_adc_result_info.channelNumber << 1;

	if (pdata->len) {
		memset(pdata->payload, 0, pdata->len);
		pdata->payload[0] = stat;
	}

	return kStatus_Success;
}

/* -----------------------------------------------------------------------
 * ADC Common Functions
 * ----------------------------------------------------------------------- */

/**
 * Initialises the opcode handlers associated with the ADC registers.
 */
int
cmd_adc_init(void)
{
	int rc;

    /* Attach FRO clock to ADC0. */
    CLOCK_Select(kADC_Clk_From_Fro);
    CLOCK_SetClkDivider(kCLOCK_DivAdcClk, 1U);

    /* Power on ADC0. */
    POWER_DisablePD(kPDRUNCFG_PD_ADC0);

#if !(defined(FSL_FEATURE_ADC_HAS_NO_CALIB_FUNC) && FSL_FEATURE_ADC_HAS_NO_CALIB_FUNC)
#if defined(FSL_FEATURE_ADC_HAS_CALIB_REG) && FSL_FEATURE_ADC_HAS_CALIB_REG
    /* Calibration after power up. */
    if (ADC_DoSelfCalibration(CMD_ADC_BASE))
#else
    uint32_t frequency;
#if defined(SYSCON_ADCCLKDIV_DIV_MASK)
    frequency = CLOCK_GetFreq(CMD_ADC_CLOCK_SOURCE) / CLOCK_GetClkDivider(kCLOCK_DivAdcClk);
#else
    frequency = CLOCK_GetFreq(CMD_ADC_CLOCK_SOURCE);
#endif /* SYSCON_ADCCLKDIV_DIV_MASK */
    /* Calibration after power up. */
    if (ADC_DoSelfCalibration(CMD_ADC_BASE, frequency))
#endif /* FSL_FEATURE_ADC_HAS_CALIB_REG */
    {
        // ADC_DoSelfCalibration Done
    }
    else
    {
        // ADC_DoSelfCalibration Failed!
    }
#endif /* FSL_FEATURE_ADC_HAS_NO_CALIB_FUNC */

    /* Configure the converter and operating mode. */
    cmd_adc_configuration();

	/* Register ADC_HI with the command registry */
	rc = cmd_register(&cmd_adchi_record);
	if (rc) {
		goto err;
	}

	/* Register ADC_LO with the command registry */
	rc = cmd_register(&cmd_adclo_record);
	if (rc) {
		goto err;
	}

	/* Register ADC_STAT with the command registry */
	rc = cmd_register(&cmd_adcstat_record);
	if (rc) {
		goto err;
	}

	return kStatus_Success;
err:
	return rc;
}

static void
cmd_adc_sample(void)
{
    ADC_DoSoftwareTriggerConvSeqA(CMD_ADC_BASE);

    /* Wait for the converter to finish. */
    while (!ADC_GetChannelConversionResult(CMD_ADC_BASE, CMD_ADC_SAMPLE_CHANNEL_NUMBER, &l_cmd_adc_result_info))
    {
    	__asm volatile("nop");
    }
}

/**
 * Configures ADC0
 */
static void
cmd_adc_configuration(void)
{
    adc_config_t adcConfigStruct;
    adc_conv_seq_config_t adcConvSeqConfigStruct;

/* Configure the converter. */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE) & FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE
    adcConfigStruct.clockMode = kADC_ClockSynchronousMode; /* Using sync clock source. */
#endif                                                     /* FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE */
    adcConfigStruct.clockDividerNumber = 1;                /* The divider for sync clock is 2. */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_RESOL) & FSL_FEATURE_ADC_HAS_CTRL_RESOL
    adcConfigStruct.resolution = kADC_Resolution12bit;
#endif /* FSL_FEATURE_ADC_HAS_CTRL_RESOL */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL) & FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL
    adcConfigStruct.enableBypassCalibration = false;
#endif /* FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_TSAMP) & FSL_FEATURE_ADC_HAS_CTRL_TSAMP
    adcConfigStruct.sampleTimeNumber = 0U;
#endif /* FSL_FEATURE_ADC_HAS_CTRL_TSAMP */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE) & FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE
    adcConfigStruct.enableLowPowerMode = false;
#endif /* FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE */
#if defined(FSL_FEATURE_ADC_HAS_TRIM_REG) & FSL_FEATURE_ADC_HAS_TRIM_REG
    adcConfigStruct.voltageRange = kADC_HighVoltageRange;
#endif /* FSL_FEATURE_ADC_HAS_TRIM_REG */
    ADC_Init(CMD_ADC_BASE, &adcConfigStruct);

#if !(defined(FSL_FEATURE_ADC_HAS_NO_INSEL) && FSL_FEATURE_ADC_HAS_NO_INSEL)
    /* Use the temperature sensor input to channel 0. */
    ADC_EnableTemperatureSensor(CMD_ADC_BASE, true);
#endif /* FSL_FEATURE_ADC_HAS_NO_INSEL. */

    /* Enable channel CMD_ADC_SAMPLE_CHANNEL_NUMBER's conversion in Sequence A. */
    adcConvSeqConfigStruct.channelMask =
        (1U << CMD_ADC_SAMPLE_CHANNEL_NUMBER); /* Includes channel CMD_ADC_SAMPLE_CHANNEL_NUMBER. */
    adcConvSeqConfigStruct.triggerMask = 0U;
    adcConvSeqConfigStruct.triggerPolarity = kADC_TriggerPolarityPositiveEdge;
    adcConvSeqConfigStruct.enableSingleStep = false;
    adcConvSeqConfigStruct.enableSyncBypass = false;
    adcConvSeqConfigStruct.interruptMode = kADC_InterruptForEachSequence;
    ADC_SetConvSeqAConfig(CMD_ADC_BASE, &adcConvSeqConfigStruct);
    ADC_EnableConvSeqA(CMD_ADC_BASE, true); /* Enable the conversion sequence A. */
    /* Clear the result register. */
    ADC_DoSoftwareTriggerConvSeqA(CMD_ADC_BASE);
    while (!ADC_GetChannelConversionResult(CMD_ADC_BASE, CMD_ADC_SAMPLE_CHANNEL_NUMBER, &l_cmd_adc_result_info))
    {
    }
    ADC_GetConvSeqAGlobalConversionResult(CMD_ADC_BASE, &l_cmd_adc_result_info);
}
