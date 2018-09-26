/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

#include <stdio.h>
#include <math.h>
#include "lcd_driver.h"
#include "retargetserial.h"
#include "gpiointerrupt.h"

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

typedef enum BT_CONN_STATE
{
	STATE_DISCONNECTED = 0,
	STATE_DISCOVER_SERVICES,
	STATE_DISCOVER_CHARACTERISTICS,
	STATE_ENABLE_NOTIFICATIONS,
	STATE_DATA_NOTIF_READY,
}BT_CONN_STATE_T;

typedef struct BT_SERVER_CONNECTION
{
	bd_addr serverBTAddress;
	BT_CONN_STATE_T connectionState;
	uint8_t connectionHandle;
	uint8_t numOfServices;
	uint8array servicesUUID[4];
	uint32_t servicesHandle[4];

}BT_SERVER_CONNECTION_T;

BT_SERVER_CONNECTION_T servers[255] = {0};
BT_CONN_STATE_T _connectionState = STATE_DISCONNECTED;

uint8array tempCharacteristic_UUID;
uint16_t tempCharacteristicHandle = 0;
float temperature_c = 0.0;
uint8_t tempData[5] = {0};

#define EVT_CONN_HANDLE(evt_type)	evt->data.evt_type.connection

#define UINT32_TO_FLT(b)         (((float)((int32_t)(b) & 0x00FFFFFFU)) * (float)pow(10,((int32_t)(b) >> 24)))


// Gecko configuration parameters (see gecko_configuration.h)
static const gecko_configuration_t config = {
  .config_flags = 0,
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .pa.config_enable = 1, // Enable high power PA
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#endif // (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
};

volatile uint8_t buttonPressEvent = 0;

uint8_t external_event = 0;

#define PB0_PRESSED (1<<0)

void buttonPushedCallback(uint8 pin)
{
	//CORE_AtomicDisableIrq();
	if(pin == BSP_BUTTON0_PIN)
	{
		external_event |= PB0_PRESSED;
		gecko_external_signal(external_event);
	}
	//CORE_AtomicEnableIrq();
}

void enable_button_interrupts()
{
	GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeInput, 0);
	GPIO_PinModeSet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, gpioModeInput, 0);

	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);

	/* configure interrupt for PB0 and PB1, both falling and rising edges */
	GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN, true, false, true);

	/* register the callback function that is invoked when interrupt occurs */
	GPIOINT_CallbackRegister(BSP_BUTTON0_PIN, buttonPushedCallback);
}

void disable_button_interrupts()
{
	GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeDisabled, 0);

	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);

	/* configure interrupt for PB0 and PB1, both falling and rising edges */
	GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN, true, false, false);

}


// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

/**
 * @brief  Main function
 */
int main(void)
{
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();

  // Initialize stack
  gecko_init(&config);

  RETARGET_SerialInit();
  RETARGET_SerialCrLf(true);

  enable_button_interrupts();

  printf("\033[2J\033[H");
  printf("*******BLE CLIENT**********\n");
  LCD_init("BLE CLIENT");

  struct gecko_msg_system_get_bt_address_rsp_t *btAddrRsp;
  char btAddress[40] = {0};
  bool waitingForConfirm = false;
  tempCharacteristic_UUID.len = 2;
  tempCharacteristic_UUID.data[0] = 0x1C;
  tempCharacteristic_UUID.data[1] = 0x2A;

  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* Check for stack event. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Do not call any stack commands before receiving the boot event.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:

    	  btAddrRsp = gecko_cmd_system_get_bt_address();
    	  LCD_write("BT ADDR", LCD_ROW_BTADDR1);
		  snprintf(btAddress, 40, "%02x:%02x:%02x:%02x:%02x:%02x",
				  btAddrRsp->address.addr[5],
				  btAddrRsp->address.addr[4],
				  btAddrRsp->address.addr[3],
				  btAddrRsp->address.addr[2],
				  btAddrRsp->address.addr[1],
				  btAddrRsp->address.addr[0]
			  );

		  printf("BT ADDR:%s\n",btAddress);
		  LCD_write(btAddress, LCD_ROW_BTADDR2);

		  if(!GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN))
		  {
			  /* delete all bondings to force the pairing process */
			  gecko_cmd_sm_delete_bondings();
			  printf("Delete all bondings\n");
		  }
		  else{
			  printf("Not pressed\n");
		  }

		  /* enable bondable to accommodate certain mobile OS */
		  gecko_cmd_sm_set_bondable_mode(1);

		  gecko_cmd_sm_configure(0x0F, sm_io_capability_displayyesno); /* Numeric comparison */

		  //connect to the server
		  bd_addr serverAddr = { .addr = {0xe7,0x30,0xef,0x57,0x0b,0x00}};
		  printf("Connecting...\n");
		  LCD_write("Connecting...", LCD_ROW_CONNECTION);
		  struct gecko_msg_le_gap_connect_rsp_t *gecko_rsp_msg = gecko_cmd_le_gap_connect(serverAddr, 0 , 1);
		  if(gecko_rsp_msg->result == 0 && gecko_rsp_msg->connection < 256){
			  servers[gecko_rsp_msg->connection].connectionHandle = gecko_rsp_msg->connection;
		  }
		  else{
			  printf("Connecting failed...\n");
			  LCD_write("Connecting failed", LCD_ROW_CONNECTION);
		  }

        break;

      case gecko_evt_le_connection_opened_id:
		//connection opened by a remote client
    	servers[EVT_CONN_HANDLE(evt_le_connection_opened)].connectionState = STATE_DISCOVER_SERVICES;
    	servers[EVT_CONN_HANDLE(evt_le_connection_opened)].connectionHandle = evt->data.evt_le_connection_opened.connection;
    	servers[EVT_CONN_HANDLE(evt_le_connection_opened)].serverBTAddress = evt->data.evt_le_connection_opened.address;
		printf("Connection Handle:%u\n", servers[EVT_CONN_HANDLE(evt_le_connection_opened)].connectionHandle);
		printf("Connection opened to remote server\n");
		printf("Server BT ADDR: %02x:%02x:%02x:%02x:%02x:%02x\n",
				servers[EVT_CONN_HANDLE(evt_le_connection_opened)].serverBTAddress.addr[5],
				servers[EVT_CONN_HANDLE(evt_le_connection_opened)].serverBTAddress.addr[4],
				servers[EVT_CONN_HANDLE(evt_le_connection_opened)].serverBTAddress.addr[3],
				servers[EVT_CONN_HANDLE(evt_le_connection_opened)].serverBTAddress.addr[2],
				servers[EVT_CONN_HANDLE(evt_le_connection_opened)].serverBTAddress.addr[1],
				servers[EVT_CONN_HANDLE(evt_le_connection_opened)].serverBTAddress.addr[0]
			  );
		LCD_write("Connected...", LCD_ROW_CONNECTION);
		if(evt->data.evt_le_connection_opened.bonding != 0xFF)
		{
			LCD_write("Connected/Bonded", LCD_ROW_CONNECTION);
			LCD_write("", LCD_ROW_PASSKEY);
			LCD_write("", LCD_ROW_ACTION);

			disable_button_interrupts();

			printf("Already bonded\n");
			uint8array thermUUID;
			thermUUID.data[0] = 0x09;
			thermUUID.data[1] = 0x18;
			thermUUID.len = 2 ;
			gecko_cmd_gatt_discover_primary_services_by_uuid(servers[EVT_CONN_HANDLE(evt_le_connection_opened)].connectionHandle,thermUUID.len, thermUUID.data);
			printf("Discovering primary services\n");
		}
		break;

      case gecko_evt_sm_bonding_failed_id:
    	  printf("Bonding failed with the client\n");
    	  LCD_write("Bonding Failed", LCD_ROW_CONNECTION);
    	  LCD_write("", LCD_ROW_PASSKEY);
		  LCD_write("", LCD_ROW_ACTION);
    	  waitingForConfirm = false;
    	  break;

      case gecko_evt_sm_bonded_id:
    	  printf("Bonding complete with the client\n");
    	  LCD_write("Connected/Bonded", LCD_ROW_CONNECTION);
    	  LCD_write("", LCD_ROW_PASSKEY);
    	  LCD_write("", LCD_ROW_ACTION);

    	  disable_button_interrupts();

    	  uint8array thermUUID;
    	  thermUUID.data[0] = 0x09;
    	  thermUUID.data[1] = 0x18;
    	  thermUUID.len = 2 ;
    	  printf("Discovering primary services\n");
    	  gecko_cmd_gatt_discover_primary_services_by_uuid(servers[EVT_CONN_HANDLE(evt_sm_bonded)].connectionHandle,thermUUID.len, thermUUID.data);
    	  break;

      case gecko_evt_sm_confirm_bonding_id:
    	  printf("Got Bonding Request. Accepting it.\n");
    	  gecko_cmd_sm_bonding_confirm(evt->data.evt_sm_confirm_bonding.connection, 1);
    	  break;

      case gecko_evt_sm_confirm_passkey_id:
    	  printf("Confirm passkey\n");
    	  printf("Do you see the same passkey on the tablet: %lu (y/n)?\n",
    			  evt->data.evt_sm_confirm_passkey.passkey);
    	  char PASSKEY[10] = {0};
    	  snprintf(PASSKEY, sizeof(PASSKEY),"KEY:%lu",evt->data.evt_sm_confirm_passkey.passkey);
    	  LCD_write(PASSKEY, LCD_ROW_PASSKEY);
    	  LCD_write("Press PB0 to confirm", LCD_ROW_ACTION);
    	  LCD_write("Bonding...", LCD_ROW_CONNECTION);
    	  printf("Press Button PB0 asap to confirm\n");
    	  waitingForConfirm = true;
    	  break;

      case gecko_evt_system_external_signal_id:

    	  printf("External Event\n");
    	  if(waitingForConfirm && (evt->data.evt_system_external_signal.extsignals & PB0_PRESSED))
    	  {
    		  CORE_AtomicDisableIrq();
    		  external_event &= ~PB0_PRESSED;
    		  CORE_AtomicEnableIrq();
    		  LCD_write("PB0 pressed", LCD_ROW_ACTION);
    		  printf("PB0 pressed: Confirming the bonding with the client...\n");
    		  gecko_cmd_sm_passkey_confirm(evt->data.evt_sm_confirm_bonding.connection, 1);
    	  }
    	  break;



      case gecko_evt_gatt_service_id:
    	  //servers[EVT_CONN_HANDLE(evt_gatt_service)].numOfServices = 0;
    	  servers[EVT_CONN_HANDLE(evt_gatt_service)].servicesHandle[servers[EVT_CONN_HANDLE(evt_gatt_service)].numOfServices] = evt->data.evt_gatt_service.service;
    	  servers[EVT_CONN_HANDLE(evt_gatt_service)].servicesUUID[servers[EVT_CONN_HANDLE(evt_gatt_service)].numOfServices].len = evt->data.evt_gatt_service.uuid.len;
    	  memcpy(
    			  &servers[EVT_CONN_HANDLE(evt_gatt_service)].servicesUUID[servers[EVT_CONN_HANDLE(evt_gatt_service)].numOfServices].data[0],
				  &evt->data.evt_gatt_service.uuid.data[0],
				  evt->data.evt_gatt_service.uuid.len);
    	  printf("Discovered Service Handle: %lu, UUID: 0x", servers[EVT_CONN_HANDLE(evt_gatt_service)].servicesHandle[servers[EVT_CONN_HANDLE(evt_gatt_service)].numOfServices] );
    	  for(int i = servers[EVT_CONN_HANDLE(evt_gatt_service)].servicesUUID[servers[EVT_CONN_HANDLE(evt_gatt_service)].numOfServices].len-1; i >= 0; i--)
    	  {
    		  printf("%02x", servers[EVT_CONN_HANDLE(evt_gatt_service)].servicesUUID[servers[EVT_CONN_HANDLE(evt_gatt_service)].numOfServices].data[i]);
    		  //printf("%02x", evt->data.evt_gatt_service.uuid.data[i]);
    	  }
    	  printf("\n");
    	  servers[EVT_CONN_HANDLE(evt_gatt_service)].numOfServices++;
    	  break;

      case gecko_evt_gatt_procedure_completed_id:
    	  switch(servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].connectionState){

    	  case STATE_DISCOVER_SERVICES:
    		  if(evt->data.evt_gatt_procedure_completed.result == 0){
				  printf("[SUCCESS] Service Discovery protocol completed\n");
				  servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].connectionState = STATE_DISCOVER_CHARACTERISTICS;
				  if(servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].servicesHandle[servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].numOfServices-1] == 0){

					  printf("[ERROR] SERVICE HANDLE INVALID\n");
					  gecko_cmd_endpoint_close(servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].connectionHandle);
					  break;

				  }
				  struct gecko_msg_gatt_discover_characteristics_by_uuid_rsp_t *rsp = gecko_cmd_gatt_discover_characteristics_by_uuid(
						  servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].connectionHandle,
						  servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].servicesHandle[servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].numOfServices-1],
						  tempCharacteristic_UUID.len,
						  tempCharacteristic_UUID.data
						  );
				  if(rsp->result != 0){
					  printf("[ERROR] Discover Characteristic. Code:%d\n",rsp->result);
					  gecko_cmd_endpoint_close(servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].connectionHandle);
				  }
			  }
			  else{
				  printf("[ERROR] Service Discovery protocol. Code: %d\n",evt->data.evt_gatt_procedure_completed.result);
				  gecko_cmd_endpoint_close(servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].connectionHandle);
			  }
    		  break;

    	  case STATE_DISCOVER_CHARACTERISTICS:
    		  if(evt->data.evt_gatt_procedure_completed.result == 0){

				 printf("[SUCCESS] Discover Service Characteristic protocol completed\n");
				 servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].connectionState = STATE_ENABLE_NOTIFICATIONS;
				 if(tempCharacteristicHandle > 0){
					printf("Setting Notifications\n");
					struct gecko_msg_gatt_set_characteristic_notification_rsp_t *rsp =
						  gecko_cmd_gatt_set_characteristic_notification(
								  servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].connectionHandle,
								  tempCharacteristicHandle,
								  gatt_indication);
					if(rsp->result != 0){
					  printf("[ERROR] Set server notification\n");
					  gecko_cmd_endpoint_close(servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].connectionHandle);
					}
				}
				 else{
					 printf("[ERROR] TEMP CHAR HANDLE: %d\n", tempCharacteristicHandle);
					 gecko_cmd_endpoint_close(servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].connectionHandle);
				 }
			 }
			 else{
				 printf("[ERROR] Discover Service Characteristic protocol\n");
				 gecko_cmd_endpoint_close(servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].connectionHandle);
			 }

    		 break;

    	  case STATE_ENABLE_NOTIFICATIONS:
    		  if(evt->data.evt_gatt_procedure_completed.result == 0)
    		  {
    			  printf("[SUCCESS] Added Notification flag in Characteristic \n");
    			  printf("START_DATA\n");
    			  servers[EVT_CONN_HANDLE(evt_gatt_procedure_completed)].connectionState = STATE_DATA_NOTIF_READY;
    		  }
    		  else{
    			  printf("[ERROR] Added Notification flag in Characteristic. Code:%u \n",evt->data.evt_gatt_procedure_completed.result);
    		  }
    		  break;
    	  default:
    		  printf("[ERROR}] Invalid State\n");
    		  break;
    	  }
		  break;

      case gecko_evt_gatt_characteristic_id:
    	  if((memcmp(&evt->data.evt_gatt_characteristic.uuid.data[0],&tempCharacteristic_UUID.data[0],tempCharacteristic_UUID.len) == 0))
    	  {
    		  printf("HEALTH THERM SERVICE: Temp Char found\n");
    		  tempCharacteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
			  printf("Temp char handle: %u\n", tempCharacteristicHandle);
			  printf("Temp char UUID: 0x%02x%02x\n",evt->data.evt_gatt_characteristic.uuid.data[1],evt->data.evt_gatt_characteristic.uuid.data[0]);
    	  }
    	  break;

      case gecko_evt_gatt_characteristic_value_id:
    	  if(evt->data.evt_gatt_characteristic_value.characteristic == tempCharacteristicHandle)
    	  {
			  if(evt->data.evt_gatt_characteristic_value.att_opcode == 0x1d){
				  struct gecko_msg_gatt_send_characteristic_confirmation_rsp_t *rsp =
						  gecko_cmd_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);
				  if(rsp->result != 0){
					  printf("[ERROR] SEND CHAR CONFIRMATION\n");
					  break;
				  }
			  }
			  memcpy(tempData, &evt->data.evt_gatt_characteristic_value.value.data[0], 5 );
			  temperature_c = UINT32_TO_FLT(*(uint32_t*)&tempData[1]);\
			  printf("\r%.02f", temperature_c);
			  char tempBuffer[10] = {0};
			  snprintf(tempBuffer, sizeof(tempBuffer), "%.02f C",temperature_c);
			  LCD_write(tempBuffer,LCD_ROW_TEMPVALUE);
    	  }
    	  break;

      case gecko_evt_le_connection_closed_id:
        /* Check if need to boot to dfu mode */
        if (boot_to_dfu) {
          /* Enter to DFU OTA mode */
          gecko_cmd_system_reset(2);
        } else {
        	/* delete all bondings to force the pairing process */
//        	gecko_cmd_sm_delete_bondings();
//        	printf("Delete all bondings\n");
        	LCD_write("Disconnected..", LCD_ROW_CONNECTION);
        	enable_button_interrupts();
        }
        break;

      /* Events related to OTA upgrading
         ----------------------------------------------------------------------------- */

      /* Check if the user-type OTA Control Characteristic was written.
       * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
      case gecko_evt_gatt_server_user_write_request_id:

        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
          /* Set flag to enter to OTA mode */
          boot_to_dfu = 1;
          /* Send response to Write Request */
          gecko_cmd_gatt_server_send_user_write_response(
            evt->data.evt_gatt_server_user_write_request.connection,
            gattdb_ota_control,
            bg_err_success);

          /* Close connection to enter to DFU OTA mode */
          gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
        }
        break;

      default:
        break;
    }
  }
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
