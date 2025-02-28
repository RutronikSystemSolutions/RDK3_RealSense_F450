/*
 * battery_monitor.h
 *
 *  Created on: 21 Apr 2023
 *      Author: jorda
 */

#ifndef BATTERY_MONITOR_BATTERY_MONITOR_H_
#define BATTERY_MONITOR_BATTERY_MONITOR_H_

#include <stdint.h>

void battery_monitor_init();

uint16_t battery_monitor_get_voltage_mv();

#endif /* BATTERY_MONITOR_BATTERY_MONITOR_H_ */
