/*
 * f450_parameters.h
 *
 *  Created on: Feb 25, 2025
 *      Author: ROJ030
 */

#ifndef F450_PARAMETERS_H_
#define F450_PARAMETERS_H_

#include <stdint.h>

#define F450_USR_NAME_LEN	10

typedef struct
{
	uint32_t msg_type;	// 0: status, 1: user database count
	char user_name[F450_USR_NAME_LEN];
	uint32_t info;
} f450_info_t;

#define F450_MSG_TYPE_AUTHENTICATE_STATUS		0
#define F450_MSG_TYPE_USER_DB_COUNT				1
#define F450_MSG_TYPE_ENROLL_STATUS				2
#define F450_MSG_TYPE_ENROLL_PROGRESS_STATUS	3
#define F450_MSG_TYPE_ENROLL_HINT_STATUS		4

#endif /* F450_PARAMETERS_H_ */

