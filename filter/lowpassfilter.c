/*
 * lowpassfilter.c
 *
 *  Created on: 27 Apr 2023
 *      Author: jorda
 */

#include "lowpassfilter.h"

void lowpassfilter_init(lowpassfilter_t* handle, float coefficient)
{
	handle->coefficient = coefficient;
	handle->initialized = 0;
	handle->buffer = 0;
}

void lowpassfilter_feed(lowpassfilter_t* handle, uint16_t value)
{
	float coeff = handle->coefficient;

	if (handle->initialized == 0)
	{
		handle->buffer = (float) value;
		handle->initialized = 1;
	}

	handle->buffer = (coeff * ((float)value)) + (1.f - coeff) * handle->buffer;
}

uint16_t lowpassfilter_get_value(lowpassfilter_t* handle)
{
	return (uint16_t) handle->buffer;
}
