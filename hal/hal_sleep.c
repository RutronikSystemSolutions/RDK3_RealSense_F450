/*
 * hal_sleep.c
 *
 *  Created on: 24 Mar 2023
 *      Author: jorda
 */

#include "hal_sleep.h"

#include "cy_syslib.h"

void hal_sleep(uint32_t ms)
{
	Cy_SysLib_Delay(ms);
}

