/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK3 RealSense F450
*               Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2023-07-21
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address:
*  Author: ROJ030, GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "hal/hal_i2c.h"

#include "host_main.h"

#include "hal_timer.h"

#include "RealSenseID/DeviceController.h"
#include "RealSenseID/FaceAuthenticator.h"

#include "f450_parameters.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

QueueHandle_t f450_to_ble_queue;
QueueHandle_t ble_to_f450_queue;

#define F450_TO_BLE_QUEUE_EL_COUNT	5
#define F450_TO_BLE_QUEUE_EL_SIZE	sizeof(f450_info_t)

#define BLE_TO_F450_QUEUE_EL_COUNT 5
#define BLE_TO_F450_QUEUE_EL_SIZE 4 // uint32_t

/**
 * Define task priorites
 * 0: iddle task
 * Maximum priority: configMAX_PRIORITIES
 */
#define ALIVE_TASK_PRIO 	1
#define REALSENSE_TASK_PRIO	2
#define BLE_TASK_PRIO		3

/**
 * Timer used to enable periodic call to the Bluetooth stack
 */
static cyhal_timer_t bluetooth_timer;

/**
 * Used to signal to the main task that an update of the Bluetooth stack has to be done
 */
static bool ble_update_pending = false;

unsigned int number_of_users = 0;

/**
 * @brief Interrupt service routine called when the timer reaches the compare value
 */
static void bluetooth_timer_isr(void *callback_arg, cyhal_timer_event_t event)
{
	(void) callback_arg;
	(void) event;
	ble_update_pending = true;
}

/**
 * @brief Initializes the timer used to update the Bluetooth stack periodically
 *
 * At the moment, an update is done every 2ms (frequency = 500 Hz)
 *
 * @retval CY_RSLT_SUCCESS success
 * @retval != CY_RSLT_SUCCESS something wrongs happened
 */
static cy_rslt_t bluetooth_timer_init(void)
{
	const uint32_t timer_frequency_hz = 10000;
	const uint32_t isr_frequency_hz = 500; // every 2ms -> ISR
	const uint8_t priority = 6;
	cyhal_timer_cfg_t configuration;
	cy_rslt_t result = CY_RSLT_SUCCESS;

	configuration.compare_value = 0;
	configuration.period = (timer_frequency_hz / isr_frequency_hz);
	configuration.direction = CYHAL_TIMER_DIR_UP;
	configuration.is_compare = false;
	configuration.is_continuous = true;
	configuration.value = 0;

	result = cyhal_timer_init(&bluetooth_timer, NC, NULL);
	if (result != CY_RSLT_SUCCESS) return result;

	result = cyhal_timer_configure(&bluetooth_timer, &configuration);
	if (result != CY_RSLT_SUCCESS) return result;

	result = cyhal_timer_set_frequency(&bluetooth_timer, timer_frequency_hz);
	if (result != CY_RSLT_SUCCESS) return result;

	cyhal_timer_register_callback(&bluetooth_timer, bluetooth_timer_isr, NULL);

	cyhal_timer_enable_event(&bluetooth_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, priority, true);

	result = cyhal_timer_start(&bluetooth_timer);
	return result;
}

/**
 * @brief Initializes the LEDs of the RDK3 board
 */
static cy_rslt_t init_leds()
{
	cy_rslt_t result = CY_RSLT_SUCCESS;

	result = cyhal_gpio_init(LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	result |= cyhal_gpio_init(LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	result |= cyhal_gpio_init(LED3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

	if (result != CY_RSLT_SUCCESS) return result;

	// Set all LEDs to OFF
	cyhal_gpio_write((cyhal_gpio_t)LED1, CYBSP_LED_STATE_OFF);
	cyhal_gpio_write((cyhal_gpio_t)LED2, CYBSP_LED_STATE_OFF);
	cyhal_gpio_write((cyhal_gpio_t)LED3, CYBSP_LED_STATE_OFF);

	return CY_RSLT_SUCCESS;
}

class MyAuthClbk : public RealSenseID::AuthenticationCallback
{
public:
    void OnResult(const RealSenseID::AuthenticateStatus status, const char* user_id) override
    {
    	printf("MyAuthClbk:OnResult : %s \r\n", Description(status));

    	f450_info_t f450_info;
    	f450_info.msg_type = F450_MSG_TYPE_AUTHENTICATE_STATUS;
    	f450_info.info = (uint32_t) status;

    	for(uint16_t i = 0; i < F450_USR_NAME_LEN; ++i) f450_info.user_name[i] = 0;

        if (status == RealSenseID::AuthenticateStatus::Success)
        {
        	// In case of success, copy the user name
        	int len = (strlen(user_id) > (F450_USR_NAME_LEN - 1)) ? (F450_USR_NAME_LEN - 1) : strlen(user_id);
        	strncpy(f450_info.user_name, user_id, len);
        	printf("User id is: %s \r\n", f450_info.user_name);
        }

        xQueueSend(f450_to_ble_queue, &f450_info, 0);
    }

    void OnHint(const RealSenseID::AuthenticateStatus hint) override
    {
        printf("MyAuthClbk:OnHint: %d \r\n", (int)hint);

        f450_info_t f450_info;
		f450_info.msg_type = F450_MSG_TYPE_AUTHENTICATE_STATUS;
		f450_info.info = (uint32_t) hint;

		xQueueSend(f450_to_ble_queue, &f450_info, 0);
    }

    void OnFaceDetected(const std::vector<RealSenseID::FaceRect>& faces, const unsigned int ts) override
    {
        // Nothing to do
    }
};

class MyEnrollClbk : public RealSenseID::EnrollmentCallback
{
public:
    void OnResult(const RealSenseID::EnrollStatus status) override
    {
    	printf("MyEnrollClbk: OnResult %s \r\n", Description(status));

    	f450_info_t f450_info;
		f450_info.msg_type = F450_MSG_TYPE_ENROLL_STATUS;
		f450_info.info = (uint32_t) status;

		xQueueSend(f450_to_ble_queue, &f450_info, 0);

		if (status == RealSenseID::EnrollStatus::Success)
		{
			// Enroll succesfull, update number of users
			number_of_users += 1;

			f450_info_t f450_info_user;
			f450_info_user.msg_type = F450_MSG_TYPE_USER_DB_COUNT;
			f450_info_user.info = (uint32_t) number_of_users;

			xQueueSend(f450_to_ble_queue, &f450_info_user, 0);
		}
    }

    void OnProgress(const RealSenseID::FacePose pose) override
    {
    	printf("MyEnrollClbk: OnProgress %s \r\n", Description(pose));

    	f450_info_t f450_info;
		f450_info.msg_type = F450_MSG_TYPE_ENROLL_PROGRESS_STATUS;
		f450_info.info = (uint32_t) pose;

		xQueueSend(f450_to_ble_queue, &f450_info, 0);
    }

    void OnHint(const RealSenseID::EnrollStatus hint) override
    {
    	printf("MyEnrollClbk: OnHint %s \r\n", Description(hint));

    	f450_info_t f450_info;
		f450_info.msg_type = F450_MSG_TYPE_ENROLL_HINT_STATUS;
		f450_info.info = (uint32_t) hint;

		xQueueSend(f450_to_ble_queue, &f450_info, 0);
    }

    void OnFaceDetected(const std::vector<RealSenseID::FaceRect>& faces, const unsigned int ts) override
    {
    	// Noting to do
    }
};

void alive_task(void * arg)
{
    (void)arg;

    for(;;)
    {
        // Toggle LED
        cyhal_gpio_toggle(LED1);
        vTaskDelay(500);
    }
}

void on_command_hook(uint8_t command, uint8_t* parameters, uint8_t parameters_len)
{
	// Add to queue and do something
	printf("on_command_hook: %d - len: %d\r\n", command, parameters_len);
	if (command == 4 && parameters_len == 1)
	{
		uint32_t totransmit = (uint32_t) parameters[0];
		xQueueSend(ble_to_f450_queue, &totransmit, 0);
	}
}

void ble_task(void* arg)
{
	(void)arg;

	// Initialize timer (Bluetooth stack update rate)
	cy_rslt_t result = bluetooth_timer_init();
	if (result != CY_RSLT_SUCCESS)
	{
		printf("Error during initialization of the timer: %lu \r\n", result);
		for(;;){}
	}

	// Initializes the bluetooth stack
	Ble_Init(on_command_hook);

	uint16_t counter = 0;

	// Main loop
	for (;;)
	{
		if (ble_update_pending)
		{
			ble_update_pending = false;
			counter++;
			if (counter > 100)
			{
				cyhal_gpio_toggle(LED3);
				counter = 0;
			}

			host_main_do();
		}

		// Something in the queue?
		f450_info_t f450_info;
		BaseType_t retval = xQueueReceive(f450_to_ble_queue, &f450_info, 0);
		if (retval == pdPASS)
		{
			// Something in the queue, add notification
			printf("Something in the F450 -> BLE queue!\r\n");

			host_main_add_notification(notification_fabric_create_for_f450(&f450_info));
		}

		vTaskDelay(portTICK_PERIOD_MS);
	}
}

void realsense_task(void* arg)
{
	(void)arg;
	static const uint32_t DO_AUTH = 1;
	static const uint32_t DO_ENROLL = 2;
	static const uint32_t DO_ERASE_ALL = 3;

	// Global init of the HALL timer (needed for the PacketManager::Timer)
	hal_timer_init();

	// Check connection with F450
	RealSenseID::DeviceController controller;
	auto status = controller.Connect({""});
	if (status != RealSenseID::Status::Ok)
	{
		printf("DeviceController: Fail to connect...\r\n");
		vTaskSuspend(NULL);
	}

	std::string version;
	status = controller.QueryFirmwareVersion(version);
	if (status != RealSenseID::Status::Ok)
	{
		printf("Fail to get firmware version...\r\n");
		vTaskSuspend(NULL);
	}
	printf("Version: %s \r\n", version.c_str());
	controller.Disconnect();

	// Init authenticator
	RealSenseID::FaceAuthenticator authenticator;
	status = authenticator.Connect({""});
	if (status != RealSenseID::Status::Ok)
	{
		printf("FaceAuthenticator: Fail to connect...\r\n");
		vTaskSuspend(NULL);
	}

	// Get the user count
	status = authenticator.QueryNumberOfUsers(number_of_users);
	if (status != RealSenseID::Status::Ok)
	{
		printf("QueryNumberOfUsers error... : %d \r\n", (int) status);
		vTaskSuspend(NULL);
	}
	printf("User count in the database: %d \r\n", number_of_users);

	MyAuthClbk auth_clbk;
	MyEnrollClbk enroll_clbk;

	for(;;)
	{
		// Something in the queue?
		uint32_t buffer = 0;
		BaseType_t retval = xQueueReceive(ble_to_f450_queue, &buffer, portMAX_DELAY);
		if (retval == pdPASS)
		{
			// Something in the queue, add notification
			printf("Something in the BLE->F450 queue! : %lu \r\n", buffer);
			if (buffer == DO_AUTH)
			{
				status = authenticator.Authenticate(auth_clbk);
				if (status != RealSenseID::Status::Ok)
				{
					printf("Authenticate error... %s \r\n", Description(status));
				}
				printf("Authenticate Done\r\n");
			}
			else if (buffer == DO_ENROLL)
			{
				char user_name[32];
				sprintf(user_name, "user_%d", number_of_users);
				printf("Do enroll of %s \r\n", user_name);
				status = authenticator.Enroll(enroll_clbk, user_name);
				if (status != RealSenseID::Status::Ok)
				{
					printf("Enroll error... %s \r\n", Description(status));
				}
				printf("Enroll done\r\n");
			}
			else if (buffer == DO_ERASE_ALL)
			{
				status = authenticator.RemoveAll();
				if (status != RealSenseID::Status::Ok)
				{
					printf("RemoveAll error... %s \r\n", Description(status));
				}
				else
				{
					printf("RemoveAll Done\r\n");
					number_of_users = 0;

					f450_info_t f450_info;
					f450_info.msg_type = F450_MSG_TYPE_USER_DB_COUNT;
					f450_info.info = (uint32_t) number_of_users;

					xQueueSend(f450_to_ble_queue, &f450_info, 0);
				}
			}
		}
	}
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    char *pcTaskName )
{
	printf("Overflow!\r\n");
}

/**
 * @brief Main function
 */
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
    	// Board init failed. Stop program execution
        CY_ASSERT(0);
    }

    // Enable global interrupts.
    __enable_irq();

    // Initialize retarget-io to use the debug UART port
    result = cy_retarget_io_init(KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
    	// retarget-io init failed. Stop program execution
        CY_ASSERT(0);
    }

    printf("------------------------- \r\n");
    printf("Starting RDK3 Real Sense Demo with RTOS v1.0\r\n");
    printf("------------------------- \r\n");

    // Initialize the User LEDs
    result = init_leds();
    if (result != CY_RSLT_SUCCESS)
	{
		printf("Error during initialization of the LEDs : %lu \r\n", result);
		for(;;){}
	}

    // Configure USER_BTN
    result = cyhal_gpio_init(USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
	if (result != CY_RSLT_SUCCESS)
	{
		printf("cyhal_gpio_init USER_BTN error\r\n");
		for(;;){}
	}

	// Create communication queues
	f450_to_ble_queue = xQueueCreate(F450_TO_BLE_QUEUE_EL_COUNT, F450_TO_BLE_QUEUE_EL_SIZE);
	if (f450_to_ble_queue == 0)
	{
		printf("Cannot create queue f450_to_ble_queue...\r\n");
		for(;;){}
	}

	ble_to_f450_queue = xQueueCreate(BLE_TO_F450_QUEUE_EL_COUNT, BLE_TO_F450_QUEUE_EL_SIZE);
	if (ble_to_f450_queue == 0)
	{
		printf("Cannot create queue ble_to_f450_queue...\r\n");
		for(;;){}
	}

    // Start the LED task
    BaseType_t rtos_retval = xTaskCreate(alive_task, "alive", configMINIMAL_STACK_SIZE, NULL, ALIVE_TASK_PRIO, NULL);
    if (rtos_retval != pdPASS)
    {
    	printf("Cannot create task alive...\r\n");
    	for(;;){}
    }

    rtos_retval = xTaskCreate(ble_task, "ble", 8192, NULL, BLE_TASK_PRIO, NULL);
	if (rtos_retval != pdPASS)
	{
		printf("Cannot create task ble...\r\n");
		for(;;){}
	}

	rtos_retval = xTaskCreate(realsense_task, "realsense", 8192, NULL, REALSENSE_TASK_PRIO, NULL);
	if (rtos_retval != pdPASS)
	{
		printf("Cannot create task realsense...\r\n");
		for(;;){}
	}

    // Start scheduler
    vTaskStartScheduler();
    printf("After scheduler call - error \r\n");
    for(;;){}

}

/* [] END OF FILE */
