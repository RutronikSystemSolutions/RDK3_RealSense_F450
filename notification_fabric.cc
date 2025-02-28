/*
 * notification_fabric.c
 *
 *  Created on: 28 Mar 2023
 *      Author: jorda
 */

#include "notification_fabric.h"

#include <stdlib.h>

static const uint8_t notification_overhead = 4;

static uint8_t compute_crc(uint8_t* data, uint8_t length)
{
	// TODO
	return 0x3;
}

void notification_fabric_free_notification(notification_t* notification)
{
	free(notification->data);
	free(notification);
}

notification_t* notification_fabric_create_for_battery_monitor(uint16_t voltage, uint8_t charge_status, uint8_t charge_fault, uint8_t dio_status)
{
	const uint8_t data_size = 5;
	const uint8_t notification_size = data_size + notification_overhead;
	const uint16_t sensor_id = 0x6;

	uint8_t* data = (uint8_t*) malloc(notification_size);

	data[0] = (uint8_t) (sensor_id & 0xFF);
	data[1] = (uint8_t) (sensor_id >> 8);

	data[2] = data_size;

	*((uint16_t*) &data[3]) = voltage;

	data[5] = charge_status;
	data[6] = charge_fault;
	data[7] = dio_status;

	data[8] = compute_crc(data, notification_size - 1);

	notification_t* retval = (notification_t*) malloc(sizeof(notification_t));
	retval->length = notification_size;
	retval->data = data;

	return retval;
}

notification_t* notification_fabric_create_for_f450(f450_info_t* info)
{
	const uint8_t data_size = 18 ; // 4 + 10 + 4
	const uint8_t notification_size = data_size + notification_overhead;
	const uint16_t sensor_id = 0xF;
	uint8_t* data = (uint8_t*) malloc(notification_size);

	data[0] = (uint8_t) (sensor_id & 0xFF);
	data[1] = (uint8_t) (sensor_id >> 8);

	data[2] = data_size;

	*((uint32_t*) &data[3]) = info->msg_type;

	for(uint16_t i = 0; i < F450_USR_NAME_LEN; ++i)
		data[7+i] = info->user_name[i];

	*((uint32_t*) &data[17]) = info->info;

	data[notification_size - 1] = compute_crc(data, notification_size - 1);

	notification_t* retval = (notification_t*) malloc(sizeof(notification_t));
	retval->length = notification_size;
	retval->data = data;

	return retval;
}
