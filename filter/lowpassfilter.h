/*
 * lowpassfilter.h
 *
 *  Created on: 27 Apr 2023
 *      Author: jorda
 */

#ifndef FILTER_LOWPASSFILTER_H_
#define FILTER_LOWPASSFILTER_H_

#include <stdint.h>


typedef struct
{
	uint8_t initialized;	/**< Is the filter initialized (0: no) ? If not, initialize it with the first call to lowpassfilter_feed */
	float coefficient;
	float buffer;
} lowpassfilter_t;


void lowpassfilter_init(lowpassfilter_t* handle, float coefficient);

void lowpassfilter_feed(lowpassfilter_t* handle, uint16_t value);

uint16_t lowpassfilter_get_value(lowpassfilter_t* handle);

#endif /* FILTER_LOWPASSFILTER_H_ */
