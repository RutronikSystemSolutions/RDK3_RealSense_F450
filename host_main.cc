/******************************************************************************
* File Name: host_main.c
*
* Version:
*
* Description:
*
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
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
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability.
*******************************************************************************/

#include "host_main.h"

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_syslib.h"
#include "cy_sysint.h"
#include "cycfg.h"
#include "cycfg_ble.h"

/******************************************************************************
* Macros
*******************************************************************************/
#define ENABLE      (1u)
#define DISABLE     (0u)

#define DEBUG_BLE_ENABLE    	DISABLE
#define DEBUG_BLE_LOGIC_ENABLE	ENABLE

#if (DEBUG_BLE_ENABLE || DEBUG_BLE_LOGIC_ENABLE)
#include <stdio.h>
#endif

#if DEBUG_BLE_ENABLE
#define DEBUG_BLE       printf
#else
#define DEBUG_BLE(...)
#endif

#if DEBUG_BLE_LOGIC_ENABLE
#define DEBUG_BLE_LOGIC       printf
#else
#define DEBUG_BLE_LOGIC(...)
#endif

#define MAX_MTU_SIZE                (512)
#define DEFAULT_MTU_SIZE            (23) 
#define NOTIFICATION_PKT_SIZE       (MAX_MTU_SIZE)
#define SUCCESS                     (0u)
#define TARGET_BDADDR       {{0xFF, 0xBB, 0xAA, 0x50, 0xA0, 0x00}, 0}
#define CUSTOM_DECL_HANDLE		cy_ble_customsConfig.attrInfo[0].customServInfo[0].customServCharDesc[0]
#define CUSTOM_CHAR_HANDLE			cy_ble_customsConfig.attrInfo[0].customServInfo[0].customServCharHandle

static host_main_t app;

static on_command_func_t on_command_hook_internal = NULL;

/*******************************************************************************
* Variables
*******************************************************************************/
/**
 * Buffer storing 512 bytes (maximum MTU size)
 * Used when sending notifications
 */
uint8 notification_buffer[NOTIFICATION_PKT_SIZE];

uint16 negotiatedMtu = DEFAULT_MTU_SIZE;
cy_stc_ble_gap_bd_addr_t    local_addr = TARGET_BDADDR;
cy_stc_ble_conn_handle_t appConnHandle;
cy_stc_ble_gatts_handle_value_ntf_t notificationPacket;
/* BLESS interrupt configuration.
 * It is used when BLE middleware operates in BLE Single CM4 Core mode. */
const cy_stc_sysint_t blessIsrCfg =
{
    /* The BLESS interrupt */
    .intrSrc      = bless_interrupt_IRQn,

    /* The interrupt priority number */
    .intrPriority = 1u
};

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
void BlessInterrupt(void);
void IasEventHandler(uint32_t event, void *eventParam);
void StackEventHandler(uint32 event, void* eventParam);
void SendNotification(void);
void SendAnswerNotification(uint16_t len, uint8_t* content);

//static uint16_t sendsomething = 0;

/*******************************************************************************
* Function Name: HostMain()
********************************************************************************
* Summary:
*  Main function for the BLE Host.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Theory:
*  The function starts BLE and UART components.
*  This function processes all BLE events
*
*******************************************************************************/

int host_main_add_notification(notification_t* notification)
{
	if(app.notification_enabled == 0)
	{
		notification_fabric_free_notification(notification);
		return -1;
	}

	if (app.mode != BLE_MODE_PUSH_DATA)
	{
		notification_fabric_free_notification(notification);
		return -1;
	}

	// Add to the list
	linked_list_add_element(&app.notification_list, (void*)notification);

	return 0;
}

int host_main_do()
{
	enum commands {
		CMD_GET_AVAILABLE_SENSORS = 0,
		CMD_START_PUSH_MODE = 1,
		CMD_STOP_PUSH_MODE = 2,
		CMD_ENABLED_DISABLE_TMF8828_8x8_MODE = 3,
		CMD_F450_COMMAND = 4
	};

    // Cy_Ble_ProcessEvents() allows BLE stack to process pending events
    Cy_BLE_ProcessEvents();

    // Any pending command to process
    if (app.cmd_to_process != 0)
    {
    	DEBUG_BLE_LOGIC("Command is : %d \r\n", app.cmd.command);

    	enum commands command = (enum commands) app.cmd.command;
    	switch(command)
    	{
			case CMD_GET_AVAILABLE_SENSORS:
				app.ack_to_send = 1;
				app.ack_len = 4;
				*((uint32_t *)app.ack_content) = (1 << 6); // Only RealSense F450
				break;

			case CMD_START_PUSH_MODE:
				app.ack_to_send = 1;
				app.ack_len = 1;
				app.ack_content[0] = app.cmd.command + 1;
				DEBUG_BLE_LOGIC("Activate push mode \r\n");
				app.mode = BLE_MODE_PUSH_DATA;
				break;

			case CMD_STOP_PUSH_MODE:
				app.ack_to_send = 1;
				app.ack_len = 1;
				app.ack_content[0] = app.cmd.command + 1;
				DEBUG_BLE_LOGIC("Activate configuration mode \r\n");
				app.mode = BLE_MODE_CONFIGURATION;
				break;

			case CMD_ENABLED_DISABLE_TMF8828_8x8_MODE:
				DEBUG_BLE_LOGIC("CMD_ENABLED_DISABLE_TMF8828_8x8_MODE param: %u \r\n", app.cmd.parameters[0]);
				break;

			case CMD_F450_COMMAND:
				DEBUG_BLE_LOGIC("CMD_F450_COMMAND param: %u \r\n", app.cmd.parameters[0]);
				if (on_command_hook_internal != NULL)
				{
					on_command_hook_internal(app.cmd.command, app.cmd.parameters, app.cmd.parameters_len);
				}
				break;
    	}

    	app.cmd_to_process = 0;
    }


    if(app.notification_enabled != 0)
    {
    	if(Cy_BLE_GATT_GetBusyStatus(appConnHandle.attId) == CY_BLE_STACK_STATE_FREE)
    	{
    		// Any acknowledge to be sent?
    		if (app.ack_to_send != 0)
    		{
    			SendAnswerNotification(app.ack_len, app.ack_content);
    			app.ack_to_send = 0;
    		}

    		// Push mode?
    		if (app.mode == BLE_MODE_PUSH_DATA)
    		{
    			// Something inside the list
    			if (app.notification_list.element_count > 0)
    			{
    				notification_t* notification = (notification_t*)app.notification_list.last->content;
    				SendAnswerNotification(notification->length, notification->data);
    				linked_list_remove_last_element(&app.notification_list);
    			}
    		}
    	}
    }

	return 0;
}

static void free_notification(void* notification)
{
	notification_t* casted = (notification_t*) notification;
	notification_fabric_free_notification(casted);
}

static void init_app()
{
	app.notification_enabled = 0;
	app.cmd_to_process = 0;
	app.mode = BLE_MODE_CONFIGURATION;
	app.ack_to_send = 0;
	app.notification_to_send = 0;

	linked_list_init(&app.notification_list, free_notification);
}

void Ble_Init(on_command_func_t on_command_hook)
{
    cy_en_ble_api_result_t apiResult;
    cy_stc_ble_stack_lib_version_t stackVersion;

    on_command_hook_internal = on_command_hook;

	init_app();

	notificationPacket.handleValPair.value.val = notification_buffer;
	notificationPacket.handleValPair.value.len = NOTIFICATION_PKT_SIZE;

    /* Initialize the BLESS interrupt */
    cy_ble_config.hw->blessIsrConfig = &blessIsrCfg;
    Cy_SysInt_Init(cy_ble_config.hw->blessIsrConfig, BlessInterrupt);

    /* Register the generic event handler */
    Cy_BLE_RegisterEventCallback(StackEventHandler);

    /* Initialize the BLE */
    Cy_BLE_Init(&cy_ble_config);

    /* Enable BLE */
    Cy_BLE_Enable();

    /* Register the IAS CallBack */
    Cy_BLE_IAS_RegisterAttrCallback(IasEventHandler);
    
    apiResult = Cy_BLE_GetStackLibraryVersion(&stackVersion);
    
    if(apiResult != CY_BLE_SUCCESS)
    {
        DEBUG_BLE("Cy_BLE_GetStackLibraryVersion API Error: 0x%2.2x \r\n",apiResult);
    }
    else
    {
        DEBUG_BLE("Stack Version: %d.%d.%d.%d \r\n", stackVersion.majorVersion, 
        stackVersion.minorVersion, stackVersion.patch, stackVersion.buildNumber);
    }
    
}

/*******************************************************************************
* Function Name: StackEventHandler()
********************************************************************************
*
* Summary:
*   This is an event callback function to receive events from the BLE Component.
*
*  event - the event code
*  *eventParam - the event parameters
*
* Return:
*   None
*
*******************************************************************************/
void StackEventHandler(uint32 event, void* eventParam)
{
    cy_en_ble_api_result_t apiResult;

    switch(event)
    {
         /* There are some events generated by the BLE component
        *  that are not required for this code example. */
        
        /**********************************************************
        *                       General Events
        ***********************************************************/
        /* This event is received when the BLE component is Started */
        case CY_BLE_EVT_STACK_ON:
        {
            DEBUG_BLE("CY_BLE_EVT_STACK_ON, Start Advertisement \r\n");    
            
            apiResult = Cy_BLE_GAP_SetBdAddress((cy_stc_ble_gap_bd_addr_t  *)&local_addr);
            
            if(apiResult != CY_BLE_SUCCESS)
            {   
                DEBUG_BLE("Cy_BLE_GAP_SetBdAddress API Error: %d \r\n",
                    apiResult);
            }
                        
             DEBUG_BLE("CUSTOM_SERV0_CHAR0_DESC0_HANDLE (%d)""\r\nCUSTOM_SERV0_CHAR0_HANDLE(%d)\r\n",CUSTOM_DECL_HANDLE, CUSTOM_CHAR_HANDLE);
            
            /* Enter into discoverable mode so that remote device can search it */
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            
            break;
        }    
        /* This event is received when there is a timeout. */
        case CY_BLE_EVT_TIMEOUT:
        {
			DEBUG_BLE("CY_BLE_EVT_TIMEOUT \r\n"); 
            
            break;
		}   
        /* This event indicates that some internal HW error has occurred. */    
		case CY_BLE_EVT_HARDWARE_ERROR:    
        {
			DEBUG_BLE("Hardware Error \r\n");
            
			break;
		}   
        /*  This event will be triggered by host stack if BLE stack is busy or 
         *  not busy. Parameter corresponding to this event will be the state 
    	 *  of BLE stack.
         *  BLE stack busy = CY_BLE_STACK_STATE_BUSY,
    	 *  BLE stack not busy = CY_BLE_STACK_STATE_FREE 
         */
    	case CY_BLE_EVT_STACK_BUSY_STATUS:
        {
			DEBUG_BLE("CY_BLE_EVT_STACK_BUSY_STATUS: %x\r\n", *(uint8 *)eventParam);
            
			break;
		}
        /* This event indicates completion of Set LE event mask. */
        case CY_BLE_EVT_LE_SET_EVENT_MASK_COMPLETE:
        {
			DEBUG_BLE("CY_BLE_EVT_LE_SET_EVENT_MASK_COMPLETE \r\n");
            
            break;
		}            
        /* This event indicates set device address command completed. */
        case CY_BLE_EVT_SET_DEVICE_ADDR_COMPLETE:
        {
            DEBUG_BLE("CY_BLE_EVT_SET_DEVICE_ADDR_COMPLETE \r\n");
            
            cy_stc_ble_events_param_generic_t *param = \
			(cy_stc_ble_events_param_generic_t *) eventParam;
        
            if( param->status != SUCCESS)
            {
                DEBUG_BLE("Failed to Set Local BDAddress [Status 0x%02X]\r\n",\
                    param->status);
               
            }
            else
            {
                DEBUG_BLE("Local Address Set successfully \r\n");
                DEBUG_BLE("BdAddress set to: %02X:%02X:%02X:%02X:%02X:%02X \r\n",\
                    local_addr.bdAddr[5],local_addr.bdAddr[4], local_addr.bdAddr[3],\
                    local_addr.bdAddr[2], local_addr.bdAddr[1], local_addr.bdAddr[0]);  
                          
                Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,\
				CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            }
            
            break;
        }
            
        /* This event indicates get device address command completed
           successfully */
        case CY_BLE_EVT_GET_DEVICE_ADDR_COMPLETE:
        {
			DEBUG_BLE("CY_BLE_EVT_GET_DEVICE_ADDR_COMPLETE: ");
            DEBUG_BLE("\n\rAdvertising with Address: ");
            for(uint8_t i = CY_BLE_GAP_BD_ADDR_SIZE; i > 0u; i--)
            {
                DEBUG_BLE("%2.2X ", ((cy_stc_ble_bd_addrs_t *)\
				((cy_stc_ble_events_param_generic_t *)eventParam)->eventParams)\
				  ->publicBdAddr[i-1]);
            }
            DEBUG_BLE("\r\n");
            
            break;
		}
        /* This event indicates set Tx Power command completed. */
        case CY_BLE_EVT_SET_TX_PWR_COMPLETE:
        {
			DEBUG_BLE("CY_BLE_EVT_SET_TX_PWR_COMPLETE \r\n");
            
            break;
		}                       
            
        /**********************************************************
        *                       GAP Events
        ***********************************************************/
       
        /* This event indicates peripheral device has started/stopped
           advertising. */
        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
        {
            DEBUG_BLE("CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP: ");
            if(Cy_BLE_GetConnectionState(appConnHandle) == CY_BLE_CONN_STATE_DISCONNECTED)
            {
                DEBUG_BLE(" <Restart ADV> \r\n");
                DEBUG_BLE("Advertising...\r\n\n");
                Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,\
                               				   CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
                app.notification_enabled = 0;
                app.mode = BLE_MODE_CONFIGURATION;
            }
           
            break;
        }
            
        /* This event is triggered instead of 'CY_BLE_EVT_GAP_DEVICE_CONNECTED', 
        if Link Layer Privacy is enabled in component customizer. */
        case CY_BLE_EVT_GAP_ENHANCE_CONN_COMPLETE:
        {
            /* BLE link is established */
            /* This event will be triggered since link layer privacy is enabled */
            DEBUG_BLE("CY_BLE_EVT_GAP_ENHANCE_CONN_COMPLETE \r\n");
            
#if DEBUG_BLE_LOGIC_ENABLE
            cy_stc_ble_gap_enhance_conn_complete_param_t *param = (cy_stc_ble_gap_enhance_conn_complete_param_t *)eventParam;
#endif
            
            DEBUG_BLE_LOGIC("Connected to Device ");

            DEBUG_BLE_LOGIC("%02X:%02X:%02X:%02X:%02X:%02X\r\n\n",param->peerBdAddr[5],\
                    param->peerBdAddr[4], param->peerBdAddr[3], param->peerBdAddr[2],\
                    param->peerBdAddr[1], param->peerBdAddr[0]);
         
            cyhal_gpio_write((cyhal_gpio_t)LED2, CYBSP_LED_STATE_ON);
            DEBUG_BLE_LOGIC("\r\nBDhandle : 0x%02X\r\n", param->bdHandle);
                
            break;
        }
        
        /* This event is triggered when there is a change to either the maximum Payload 
        length or the maximum transmission time of Data Channel PDUs in either direction */
        case CY_BLE_EVT_DATA_LENGTH_CHANGE:
        {
            DEBUG_BLE("CY_BLE_EVT_DATA_LENGTH_CHANGE \r\n");
            cy_stc_ble_set_phy_info_t phyParam;
            
            /* Configure the BLE Component for 2Mbps data rate */
            phyParam.bdHandle = appConnHandle.bdHandle;
            phyParam.allPhyMask = CY_BLE_PHY_NO_PREF_MASK_NONE;
            phyParam.phyOption = 0;
            phyParam.rxPhyMask = CY_BLE_PHY_MASK_LE_2M;
            phyParam.txPhyMask = CY_BLE_PHY_MASK_LE_2M;
            
            Cy_BLE_EnablePhyUpdateFeature();
            apiResult = Cy_BLE_SetPhy(&phyParam);
            if(apiResult != CY_BLE_SUCCESS)
            {
                DEBUG_BLE("Failed to set PHY..[bdHandle 0x%02X] : 0x%4x\r\n",
                    phyParam.bdHandle, apiResult);
            }
            else
            {
                DEBUG_BLE("Setting PHY.[bdHandle 0x%02X] \r\n", phyParam.bdHandle);
            }
            
            break;
        }
        
        /* This event is generated at the GAP Peripheral end after connection 
           is completed with peer Central device. */
        case CY_BLE_EVT_GAP_DEVICE_CONNECTED: 
		{
			DEBUG_BLE("CY_BLE_EVT_GAP_DEVICE_CONNECTED \r\n");
			app.notification_enabled = 0;
                      
            break;
		}            
        /* This event is generated when disconnected from remote device or 
           failed to establish connection. */
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:   
		{
			if(Cy_BLE_GetConnectionState(appConnHandle) == CY_BLE_CONN_STATE_DISCONNECTED)
            {
                DEBUG_BLE("CY_BLE_EVT_GAP_DEVICE_DISCONNECTED %d\r\n",\
                    CY_BLE_CONN_STATE_DISCONNECTED);
            }
            
            /* Device disconnected; restart advertisement */
            negotiatedMtu = DEFAULT_MTU_SIZE;
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,\
                CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            
            break;
		}            
        /* This event is generated at the GAP Central and the peripheral end 
           after connection parameter update is requested from the host to 
           the controller. */
        case CY_BLE_EVT_GAP_CONNECTION_UPDATE_COMPLETE:
		{
			DEBUG_BLE("CY_BLE_EVT_GAP_CONNECTION_UPDATE_COMPLETE \r\n");
            
            break;
		}
        /* This event indicates completion of the Cy_BLE_SetPhy API*/
		case CY_BLE_EVT_SET_PHY_COMPLETE:
        {
            DEBUG_BLE("Updating the Phy.....\r\n");
            cy_stc_ble_events_param_generic_t * param =\
            (cy_stc_ble_events_param_generic_t *) eventParam;
            if(param->status ==SUCCESS)
            {
                DEBUG_BLE("SET PHY updated to 2 Mbps\r\n");
                Cy_BLE_GetPhy(appConnHandle.bdHandle);
            }
            else
            {
                DEBUG_BLE("SET PHY Could not update to 2 Mbps\r\n");
                Cy_BLE_GetPhy(appConnHandle.bdHandle);
            }
            
            break;
        }
        /* This event indicates completion of the Cy_BLE_GetPhy API */
        case CY_BLE_EVT_GET_PHY_COMPLETE:
        {
            /* To remove unused parameter warning when UART debug is disabled */
            #if (DEBUG_BLE == ENABLE)
            cy_stc_ble_events_param_generic_t *param =\
            cy_stc_ble_events_param_generic_t *)eventParam;
            cy_stc_ble_phy_param_t *phyparam = NULL;
                      
            if(param->status == SUCCESS)
            {
                phyparam = (cy_stc_ble_phy_param_t *)param->eventParams;
                DEBUG_BLE("RxPhy Mask : 0x%02X\r\nTxPhy Mask : 0x%02X\r\n", \
                    phyparam->rxPhyMask, phyparam->txPhyMask);            
            }
            #endif
            
            break;

        }
        /* This event indicates that the controller has changed the transmitter
           PHY or receiver PHY in use */
        case CY_BLE_EVT_PHY_UPDATE_COMPLETE:
        {
            DEBUG_BLE("UPDATE PHY parameters\r\n");
             /* To remove unused parameter warning when UART debug is disabled */
            #if (DEBUG_BLE == ENABLE)
            cy_stc_ble_events_param_generic_t *param =\
           (cy_stc_ble_events_param_generic_t *)eventParam;
            cy_stc_ble_phy_param_t *phyparam = NULL;
            if(param->status == SUCCESS)
            {
                phyparam = (cy_stc_ble_phy_param_t *)param->eventParams;
                DEBUG_BLE("RxPhy Mask : 0x%02X\r\nTxPhy Mask : 0x%02X\r\n",\
                    phyparam->rxPhyMask, phyparam->txPhyMask);            
            }
            #endif
            
            break;
        }		
            
        /**********************************************************
        *                       GATT Events
        ***********************************************************/
            
        /* This event is generated at the GAP Peripheral end after connection 
           is completed with peer Central device. */
        case CY_BLE_EVT_GATT_CONNECT_IND:
        {
            appConnHandle = *(cy_stc_ble_conn_handle_t *)eventParam;
            DEBUG_BLE("CY_BLE_EVT_GATT_CONNECT_IND: %x, %x \r\n", 
                        appConnHandle.attId, 
                        appConnHandle.bdHandle);
            
            /* Update Notification packet with the data. */
            notificationPacket.connHandle = appConnHandle;
            notificationPacket.handleValPair.attrHandle = CUSTOM_CHAR_HANDLE;
            
            Cy_BLE_GetPhy(appConnHandle.bdHandle);                   
           
            break;
        }    
        /* This event is generated at the GAP Peripheral end after disconnection. */
        case CY_BLE_EVT_GATT_DISCONNECT_IND:
        {
            DEBUG_BLE("CY_BLE_EVT_GATT_DISCONNECT_IND \r\n");
            if(appConnHandle.bdHandle == (*(cy_stc_ble_conn_handle_t *)eventParam).bdHandle)
            {
            	DEBUG_BLE_LOGIC("Disconnected. \r\n\n");
                appConnHandle.bdHandle = CY_BLE_INVALID_CONN_HANDLE_VALUE;
                appConnHandle.attId    = CY_BLE_INVALID_CONN_HANDLE_VALUE;
            }
            cyhal_gpio_write((cyhal_gpio_t)LED2, CYBSP_LED_STATE_OFF);
            
            break;
        }
        /* This event is triggered when 'GATT MTU Exchange Request' 
           received from GATT client device. */
        case CY_BLE_EVT_GATTS_XCNHG_MTU_REQ:
        {
            negotiatedMtu = (((cy_stc_ble_gatt_xchg_mtu_param_t *)eventParam)->mtu < CY_BLE_GATT_MTU) ?
                            ((cy_stc_ble_gatt_xchg_mtu_param_t *)eventParam)->mtu : CY_BLE_GATT_MTU;
            DEBUG_BLE("CY_BLE_EVT_GATTS_XCNHG_MTU_REQ negotiated = %d\r\n", negotiatedMtu);
            
            break;
        }   
        /* This event is triggered when there is a write request from the 
           Client device */
        case CY_BLE_EVT_GATTS_WRITE_REQ:
        {
        	DEBUG_BLE_LOGIC("StackEventHandler CY_BLE_EVT_GATTS_WRITE_REQ. Event : %ld \r\n", event);

            DEBUG_BLE("CY_BLE_EVT_GATTS_WRITE_REQ \r\n");  
            cy_stc_ble_gatt_write_param_t *write_req_param = \
            (cy_stc_ble_gatt_write_param_t *)eventParam;
            cy_stc_ble_gatts_db_attr_val_info_t attr_param;
                        
            DEBUG_BLE("Received GATT Write Request [bdHandle %02X]\r\n",\
                write_req_param->connHandle.bdHandle);
            attr_param.connHandle.bdHandle = write_req_param->connHandle.bdHandle;
            attr_param.connHandle.attId = write_req_param->connHandle.attId;
            attr_param.flags = CY_BLE_GATT_DB_PEER_INITIATED;
            attr_param.handleValuePair = write_req_param->handleValPair;
            attr_param.offset = 0;
            
            DEBUG_BLE("write_req_param->handleValPair.attrHandle = %d",\
                write_req_param->handleValPair.attrHandle);
            DEBUG_BLE(" -> Should be = CUSTOM_SERV0_CHAR0_DESC0_HANDLE (%d)\r\n-> Should be = CUSTOM_SERV0_CHAR0_HANDLE(%d)\r\n",CUSTOM_DECL_HANDLE, CUSTOM_CHAR_HANDLE);
                  
           
			if(write_req_param->handleValPair.attrHandle == (CUSTOM_DECL_HANDLE))
            {
                if(Cy_BLE_GATTS_WriteRsp(write_req_param->connHandle) != CY_BLE_SUCCESS)
                {
                    DEBUG_BLE("Failed to send write response \r\n");
                    DEBUG_BLE_LOGIC("Error... \r\n");
                }
                else
                {
                    DEBUG_BLE("GATT write response sent \r\n");
                    app.notification_enabled = attr_param.handleValuePair.value.val[0];
                    DEBUG_BLE("app.notification_enabled = %d\r\n", app.notification_enabled);
                    DEBUG_BLE_LOGIC("Notification Enabled.\r\n\n");
                    notificationPacket.connHandle = appConnHandle;
                    notificationPacket.handleValPair.attrHandle = CUSTOM_CHAR_HANDLE;
                }

                DEBUG_BLE_LOGIC("Allright \r\n");
            }
			else if(write_req_param->handleValPair.attrHandle == (CUSTOM_CHAR_HANDLE))
			{
				if(Cy_BLE_GATTS_WriteRsp(write_req_param->connHandle) != CY_BLE_SUCCESS)
				{
					DEBUG_BLE("Failed to send write response \r\n");
					DEBUG_BLE("Error... \r\n");
				}
				else
				{
					DEBUG_BLE_LOGIC("Allright, sent ACK \r\n");
					uint16_t len = write_req_param->handleValPair.value.len;
					if (len > 0)
					{
						// First byte is the command, then the parameters
						app.cmd.command = write_req_param->handleValPair.value.val[0];
						app.cmd.parameters_len = len - 1;

						for(uint16_t i = 0; (i < len - 1) && (i < BLE_CMD_PARAM_MAX_SIZE); ++i)
							app.cmd.parameters[i] = write_req_param->handleValPair.value.val[i + 1];

						DEBUG_BLE_LOGIC("len is: %u \r\n", len);

						app.cmd_to_process = 1;
					}
					else
					{
						DEBUG_BLE_LOGIC("Not a valid command. Length is 0 ... \r\n");
					}
				}

			}
			else
			{
				DEBUG_BLE_LOGIC("Not correct is %d but should be %d \r\n", write_req_param->handleValPair.attrHandle, CUSTOM_DECL_HANDLE);
			}
            break;
        }
        case CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ:
        {
            DEBUG_BLE("CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ \r\n");  
            break;
        }
        /**********************************************************
        *                       Other Events
        ***********************************************************/
        default:
            DEBUG_BLE("OTHER event: %lx \r\n", (unsigned long) event);
			break;
        
    }
}

/*******************************************************************************
* Function Name: SendNotification()
********************************************************************************
* Summary:
* Sends notification data to the GATT Client
*
* Parameters:
* None
*
* Return:
* None
*
*******************************************************************************/
void SendNotification(void)
{
    cy_en_ble_api_result_t apiResult;
    apiResult = Cy_BLE_GATTS_Notification(&notificationPacket);
    if(apiResult == CY_BLE_ERROR_INVALID_PARAMETER)
    {
        DEBUG_BLE("Couldn't send notification. [CY_BLE_ERROR_INVALID_PARAMETER]\r\n");
    }
    else if(apiResult != CY_BLE_SUCCESS)
    {
        DEBUG_BLE("Attrhandle = 0x%4X  Cy_BLE_GATTS_Notification API Error:"\
            "0x%2.2x \r\n", notificationPacket.handleValPair.attrHandle, apiResult);
    }
}

void SendAnswerNotification(uint16_t len, uint8_t* content)
{
    cy_en_ble_api_result_t apiResult;
    notificationPacket.handleValPair.value.len = len;

    for(uint16 i = 0; i < len; ++i)
    	notificationPacket.handleValPair.value.val[i] = content[i];

    	//*(uint32_t*)notificationPacket.handleValPair.value.val = tosend;

    apiResult = Cy_BLE_GATTS_Notification(&notificationPacket);
    if(apiResult == CY_BLE_ERROR_INVALID_PARAMETER)
    {
        DEBUG_BLE("Couldn't send notification. [CY_BLE_ERROR_INVALID_PARAMETER]\r\n");
    }
    else if(apiResult != CY_BLE_SUCCESS)
    {
        DEBUG_BLE("Attrhandle = 0x%4X  Cy_BLE_GATTS_Notification API Error:"\
            "0x%2.2x \r\n", notificationPacket.handleValPair.attrHandle, apiResult);
    }
    // Set back
    notificationPacket.handleValPair.value.len = NOTIFICATION_PKT_SIZE;
}

/*******************************************************************************
* Function Name: BlessInterrupt
********************************************************************************
* BLESS ISR
* It is used used when BLE middleware operates in BLE single CM4
*
*******************************************************************************/
/* BLESS ISR */
void BlessInterrupt(void)
{
    /* Call interrupt processing */
    Cy_BLE_BlessIsrHandler();
}

void IasEventHandler(uint32_t event, void *eventParam)
{
    (void) eventParam;
}

/* [] END OF FILE */
