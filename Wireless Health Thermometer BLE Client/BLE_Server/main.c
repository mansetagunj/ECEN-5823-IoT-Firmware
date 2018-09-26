/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Thermometer Example Application
 *
 * This Thermometer and OTA example allows the user to measure temperature
 * using the temperature sensor on the WSTK. The values can be read with the
 * Health Thermometer reader on the Blue Gecko smartphone app.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silicon Labs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board Headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "infrastructure.h"

/* GATT database */
#include "gatt_db.h"

/* EM library (EMlib) */
#include "em_system.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include "em_core.h"

/* Device initialization header */
#include "hal-config.h"

#ifdef FEATURE_BOARD_DETECTED
#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif
#else
#error This sample app only works with a Silicon Labs Board
#endif

#include <stdio.h>
#include "i2cspm.h"
#include "si7013.h"
#include "tempsens.h"
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

/* Gecko configuration parameters (see gecko_configuration.h) */
#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

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

/* Flag for indicating DFU Reset must be performed */
uint8_t boot_to_dfu = 0;

/**
 * @brief Function for taking a single temperature measurement with the WSTK Relative Humidity and Temperature (RHT) sensor.
 */
void temperatureMeasure()
{
  uint8_t htmTempBuffer[5]; /* Stores the temperature data in the Health Thermometer (HTM) format. */
  uint8_t flags = 0x00;   /* HTM flags set as 0 for Celsius, no time stamp and no temperature type. */
  int32_t tempData;     /* Stores the Temperature data read from the RHT sensor. */
  uint32_t rhData = 0;    /* Dummy needed for storing Relative Humidity data. */
  uint32_t temperature;   /* Stores the temperature data read from the sensor in the correct format */
  uint8_t *p = htmTempBuffer; /* Pointer to HTM temperature buffer needed for converting values to bitstream. */
  static char LCD_TEMP[10] = {0};
  /* Convert flags to bitstream and append them in the HTM temperature data buffer (htmTempBuffer) */
  UINT8_TO_BITSTREAM(p, flags);

  /* Sensor relative humidity and temperature measurement returns 0 on success, nonzero otherwise */
  if (Si7013_MeasureRHAndTemp(I2C0, SI7021_ADDR, &rhData, &tempData) == 0) {
    /* Convert sensor data to correct temperature format */
    temperature = FLT_TO_UINT32(tempData, -3);
    /* Convert temperature to bitstream and place it in the HTM temperature data buffer (htmTempBuffer) */
    UINT32_TO_BITSTREAM(p, temperature);
    /* Send indication of the temperature in htmTempBuffer to all "listening" clients.
     * This enables the Health Thermometer in the Blue Gecko app to display the temperature.
     *  0xFF as connection ID will send indications to all connections. */
    gecko_cmd_gatt_server_send_characteristic_notification(
      0xFF, gattdb_temperature_measurement, 5, htmTempBuffer);
    snprintf(LCD_TEMP, sizeof(LCD_TEMP),"%0.2f",(float)tempData/1000);
    LCD_write(LCD_TEMP, LCD_ROW_TEMPVALUE);
    //printf("Temp: %f: %s\n", (float)tempData/1000,LCD_TEMP);
  }
}

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

void enable_button_interrupts(void)
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
  // Initialize the Temperature Sensor
  Si7013_Detect(I2C0, SI7021_ADDR, NULL);

  RETARGET_SerialInit();
  RETARGET_SerialCrLf(true);

  enable_button_interrupts();

  printf("\033[2J\033[H");
  printf("********BLE Server*********\n");
  LCD_init("BLE Server");

  struct gecko_msg_system_get_bt_address_rsp_t *btAddrRsp;
  char btAddress[40] = {0};
  bool waitingForConfirm = false;
  bool bonding_complete = false;

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
    	  snprintf(btAddress, 40, "%02x:%02x:%02x:%02x:%02x:%02x",
				  btAddrRsp->address.addr[5],
				  btAddrRsp->address.addr[4],
				  btAddrRsp->address.addr[3],
				  btAddrRsp->address.addr[2],
				  btAddrRsp->address.addr[1],
				  btAddrRsp->address.addr[0]
			  );

    	  LCD_write("BT ADDR", LCD_ROW_BTADDR1);
    	  printf("%s",btAddress);
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

		  /* Set advertising parameters. 100ms advertisement interval.
		   * The first two parameters are minimum and maximum advertising interval, both in
		   * units of (milliseconds * 1.6). */
		  gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);

		  /* Start general advertising and enable connections. */
		  gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable,
				  le_gap_connectable_scannable);
		  break;

      case gecko_evt_le_connection_opened_id:
    	  //connection opened by a remote client
    	  gecko_cmd_sm_increase_security(evt->data.evt_le_connection_opened.connection);

    	  printf("Connection opened id from client. Client addr: %02x:%02x:%02x:%02x:%02x:%02x\n",
    			  evt->data.evt_le_connection_opened.address.addr[5],
				  evt->data.evt_le_connection_opened.address.addr[4],
				  evt->data.evt_le_connection_opened.address.addr[3],
				  evt->data.evt_le_connection_opened.address.addr[2],
				  evt->data.evt_le_connection_opened.address.addr[1],
				  evt->data.evt_le_connection_opened.address.addr[0]
    	  	  	  );
    	  char buffer[25] = {0};
    	  snprintf(buffer, 25, "Client: %02x:%02x",evt->data.evt_le_connection_opened.address.addr[1],evt->data.evt_le_connection_opened.address.addr[0]);
    	  LCD_write(buffer, LCD_ROW_CLIENTADDR);
    	  break;

      case gecko_evt_le_connection_parameters_id:
    	  if(evt->data.evt_le_connection_parameters.security_mode >= le_connection_mode1_level3){
    		  bonding_complete = true;
    		  LCD_write("Connected/Bonded", LCD_ROW_CONNECTION);
			  LCD_write("", LCD_ROW_PASSKEY);
			  LCD_write("", LCD_ROW_ACTION);
    		  printf("Already bonded with level: %x\n",evt->data.evt_le_connection_parameters.security_mode);
    	  }
    	  break;

      case gecko_evt_sm_bonding_failed_id:
    	  printf("Bonding failed with the client: %u\n", evt->data.evt_sm_bonding_failed.reason);
    	  static char bondingFailed[15]= {0};
    	  snprintf(bondingFailed, 10, "Bond Fail:%u",evt->data.evt_sm_bonding_failed.reason);
    	  LCD_write(bondingFailed, LCD_ROW_CONNECTION);
    	  LCD_write("", LCD_ROW_PASSKEY);
		  LCD_write("", LCD_ROW_ACTION);
    	  waitingForConfirm = false;
    	  bonding_complete = false;
    	  gecko_cmd_le_connection_close(evt->data.evt_sm_bonding_failed.connection);
    	  break;

      case gecko_evt_sm_bonded_id:
    	  printf("Bonding complete with the client\n");
    	  LCD_write("Connected/Bonded", LCD_ROW_CONNECTION);
    	  LCD_write("", LCD_ROW_PASSKEY);
    	  LCD_write("", LCD_ROW_ACTION);
    	  disable_button_interrupts();
    	  bonding_complete = true;
    	  break;

      case gecko_evt_sm_confirm_passkey_id:
    	  printf("Confirm passkey\n");
    	  printf("Do you see the same passkey on the tablet: %lu (y/n)?\n",
    			  evt->data.evt_sm_confirm_passkey.passkey);
    	  char PASSKEY[20] = {0};
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
    		  CORE_ATOMIC_IRQ_DISABLE();
    		  external_event &= ~PB0_PRESSED;
    		  CORE_ATOMIC_IRQ_ENABLE();
    		  printf("PB0 pressed: Confirming the bonding with the client...\n");
    		  LCD_write("PB0 pressed", LCD_ROW_ACTION);
    		  gecko_cmd_sm_passkey_confirm(evt->data.evt_sm_confirm_bonding.connection, 1);
    	  }
    	  break;

    	  //case when the passkey is displayed to the user
      case gecko_evt_sm_passkey_display_id:
    	  printf("Passkey: %lu\n", evt->data.evt_sm_passkey_display.passkey);
    	  break;

      /* This event is generated when a connected client has either
       * 1) changed a Characteristic Client Configuration, meaning that they have enabled
       * or disabled Notifications or Indications, or
       * 2) sent a confirmation upon a successful reception of the indication. */
      case gecko_evt_gatt_server_characteristic_status_id:
    	  //printf("Server Char status id evt\n");
    	  //printf("EVT Char: %u\n", evt->data.evt_gatt_server_characteristic_status.characteristic);
    	  //printf("EVT status flag: %u\n", evt->data.evt_gatt_server_characteristic_status.status_flags);
    	  //printf("EVT client config flag: %u\n", evt->data.evt_gatt_server_characteristic_status.client_config_flags);
        /* Check that the characteristic in question is temperature - its ID is defined
         * in gatt.xml as "temperature_measurement". Also check that status_flags = 1, meaning that
         * the characteristic client configuration was changed (notifications or indications
         * enabled or disabled). */
        if ((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement)
            && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x01)) {
          if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x02) {
            /* Indications have been turned ON - start the repeating timer. The 1st parameter '32768'
             * tells the timer to run for 1 second (32.768 kHz oscillator), the 2nd parameter is
             * the timer handle and the 3rd parameter '0' tells the timer to repeat continuously until
             * stopped manually.*/
        	  //printf("Starting Temp Timer\n");
            gecko_cmd_hardware_set_soft_timer(32768, 0, 0);
          } else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x00) {
            /* Indications have been turned OFF - stop the timer. */
            gecko_cmd_hardware_set_soft_timer(0, 0, 0);
          }
        }
        break;

      /* This event is generated when the software timer has ticked. In this example the temperature
       * is read after every 1 second and then the indication of that is sent to the listening client. */
      case gecko_evt_hardware_soft_timer_id:
        /* Measure the temperature as defined in the function temperatureMeasure() */
    	  if(bonding_complete)
    	  {
    		  temperatureMeasure();
    	  }
        break;

      case gecko_evt_le_connection_closed_id:
        /* Check if need to boot to dfu mode */
        if (boot_to_dfu) {
          /* Enter to DFU OTA mode */
          gecko_cmd_system_reset(2);
        } else {
      	  waitingForConfirm = false;
      	  bonding_complete = false;
          /* Stop timer in case client disconnected before indications were turned off */
          LCD_write("Client disconnected", LCD_ROW_CONNECTION);
          LCD_write("", LCD_ROW_CLIENTADDR);
          printf("Client Disconnected\n");
          gecko_cmd_hardware_set_soft_timer(0, 0, 0);
          enable_button_interrupts();
          /* delete all bondings to force the pairing process */
//          gecko_cmd_sm_delete_bondings();
//          printf("Delete all bondings\n");
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        }
        break;

      /* Events related to OTA upgrading
         ----------------------------------------------------------------------------- */

      /* Checks if the user-type OTA Control Characteristic was written.
       * If written, boots the device into Device Firmware Upgrade (DFU) mode. */
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
