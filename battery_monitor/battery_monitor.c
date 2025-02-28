/*
 * battery_monitor.c
 *
 *  Created on: 21 Apr 2023
 *      Author: jorda
 */


#include "battery_monitor.h"

#include "cyhal_adc.h"
#include "cycfg_pins.h"

/****
 * ADC
 */
/* ADC Object */
cyhal_adc_t adc_obj;

#define ARD_CH_NUM 1

/* ADC Channel Objects */
cyhal_adc_channel_t adc_channels[ARD_CH_NUM];

/* Default ADC configuration */
const cyhal_adc_config_t adc_config =
{
        .continuous_scanning=true,  	// Continuous Scanning is enabled
        .average_count=32,           	// Average count disabled
        .vref=CYHAL_ADC_REF_VDDA,   	// VREF for Single ended channel set to VDDA
        .vneg=CYHAL_ADC_VNEG_VSSA,  	// VNEG for Single ended channel set to VSSA
        .resolution = 12u,          	// 12-bit resolution
        .ext_vref = NC,
        .bypass_pin = NC,
};

/* ADC channel configuration */
const cyhal_adc_channel_config_t channel_config =
{
		.enable_averaging = false,  // Disable averaging for channel
        .min_acquisition_ns = 10000, // Minimum acquisition time set to 10us
        .enabled = true
};

cy_rslt_t app_hw_init(void)
{
	cy_rslt_t result;

	/* Initialize the Arduino ADC Block.*/
	cyhal_adc_free(&adc_obj);
	result = cyhal_adc_init(&adc_obj, ARDU_ADC1, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
    	goto return_err;
    }

    /* Initialize a channel A5 and configure it to scan P10_5 in single ended mode. */
    result  = cyhal_adc_channel_init_diff(&adc_channels[0], &adc_obj, ARDU_ADC6, CYHAL_ADC_VNEG, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {goto return_err;}

    /* Update ADC configuration */
    result = cyhal_adc_configure(&adc_obj, &adc_config);
    if(result != CY_RSLT_SUCCESS)
    {goto return_err;}

	return_err:
	return result;
}

void battery_monitor_init()
{
	if (app_hw_init() != CY_RSLT_SUCCESS) return;

	// Enable measurement
	if (cyhal_gpio_init( DIV_EN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 0) != CY_RSLT_SUCCESS) return;
	cyhal_gpio_write(DIV_EN, 1);
}

uint16_t battery_monitor_get_voltage_mv()
{
	float retval = cyhal_adc_read_uv(&adc_channels[0]) / 1000;
	// retval = retval * 2.5f / 3.3f; -> If using 2.5V

	retval = retval * 2; // Voltage divider

	return (uint16_t) retval;
}
