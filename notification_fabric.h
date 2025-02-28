/*
 * notification_fabric.h
 *
 *  Created on: 28 Mar 2023
 *      Author: jorda
 */

#ifndef NOTIFICATION_FABRIC_H_
#define NOTIFICATION_FABRIC_H_

#include <stdint.h>
#include "f450_parameters.h"

typedef struct
{
	uint8_t length;
	uint8_t* data;
} notification_t;

void notification_fabric_free_notification(notification_t* notification);

notification_t* notification_fabric_create_for_battery_monitor(uint16_t voltage, uint8_t charge_status, uint8_t charge_fault, uint8_t dio_status);

notification_t* notification_fabric_create_for_f450(f450_info_t* info);

#endif /* NOTIFICATION_FABRIC_H_ */
