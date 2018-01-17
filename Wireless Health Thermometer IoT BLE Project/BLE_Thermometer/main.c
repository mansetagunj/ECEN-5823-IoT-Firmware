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
#include "boards.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "aat.h"
#include "infrastructure.h"

/* GATT database */
#include "gatt_db.h"

/* EM library (EMlib) */
#include "em_system.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#ifdef FEATURE_BOARD_DETECTED
#include "bspconfig.h"
#include "pti.h"
#else
#error This sample app only works with a Silicon Labs Board
#endif

#ifdef FEATURE_IOEXPANDER
#include "bsp.h"
#include "bsp_stk_ioexp.h"
#endif /* FEATURE_IOEXPANDER */

/* Device initialization header */
#include "InitDevice.h"

/* Temperature sensor and I2c*/
#include "i2cspmconfig.h"
#include "i2cspm.h"

#include "I2C_TempSensor.h"
#include "timer0.h"
#include "common.h"
#include "letimer0.h"
#include "adc0.h"
#include "spi.h"
#include "cmu.h"
#include "stdio.h"
#include "retargetserial.h"

#ifdef FEATURE_SPI_FLASH
#include "em_usart.h"
#include "mx25flash_spi.h"
#endif /* FEATURE_SPI_FLASH */

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

#ifdef FEATURE_PTI_SUPPORT
static const RADIO_PTIInit_t ptiInit = RADIO_PTI_INIT;
#endif

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
  #ifdef FEATURE_PTI_SUPPORT
  .pti = &ptiInit,
  #endif
};

/* Flag for indicating DFU Reset must be performed */
uint8_t boot_to_dfu = 0;
volatile uint32_t g_external_event_status = 0;
volatile uint32_t g_temp_ext_event_status = 0;
/**
 * @brief Function for taking a single temperature measurement with the WSTK Relative Humidity and Temperature (RHT) sensor.
 */
void temperatureMeasureAndSend()
{
	uint8_t tempBuffer[14] = {0}; /* Stores the temperature data in the Health Thermometer (HTM) format. */
	uint8_t flags = 0x04;/* HTM flags set as 0 for Celsius, no time stamp and yes temperature type. */
	float /*int16_t*/ tempValue = 0;     /* Stores the Temperature data read from the RHT sensor. */
	uint8_t *p = tempBuffer; /* Pointer to HTM temperature buffer needed for converting values to bitstream. */

	/* Convert flags to bitstream and append them in the HTM temperature data buffer (htmTempBuffer) */
	UINT8_TO_BITSTREAM(p, flags);
	if(I2C_TempSensor_isEnable())
	{
	int I2C_Status = I2C_TempSensor_GetTempSi7021(&tempValue);

	if(I2C_Status == 0)	//i2cTransferDone has a value of 0
	{
		if(tempValue < g_temperatureThreshold)
		{
			GPIO_PinOutSet(LED1_port,LED1_pin);
		}
		/* Convert temperature to bitstream and place it in the HTM temperature data buffer (htmTempBuffer) */
		UINT32_TO_BITSTREAM(p, (uint32_t)FLT_TO_UINT32((tempValue*100),-2));
		printf("Temperature: %02f\r\n",tempValue);
	}
	UINT16_TO_BITSTREAM(p, 0);
	UINT8_TO_BITSTREAM(p, 0);
	UINT8_TO_BITSTREAM(p, 0);
	UINT8_TO_BITSTREAM(p, 0);
	UINT8_TO_BITSTREAM(p, 0);
	UINT8_TO_BITSTREAM(p, 0);
	UINT8_TO_BITSTREAM(p,3);
	gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_temp_measurement, 14, tempBuffer);
	}

}
#define FLASH_TEMP_DATA_KEY (0X4000)
#define FLASH_TEMP_DATA_ARRAY_LEN 4
union {
	float value_float;
	uint8_t	value_array[FLASH_TEMP_DATA_ARRAY_LEN];
}flash_data_temp_set;
//flash_data_temp_set_t flash_data;
/**
 * @brief  Main function
 */
int main(void)
{
#ifdef FEATURE_SPI_FLASH
  /* Put the SPI flash into Deep Power Down mode for those radio boards where it is available */
  MX25_init();
  MX25_DP();
  /* We must disable SPI communication */
  USART_Reset(USART1);

#endif /* FEATURE_SPI_FLASH */

  /* Initialize peripherals */
  enter_DefaultMode_from_RESET();

  /* Initialize stack */
  gecko_init(&config);

#ifdef FEATURE_IOEXPANDER
  if ( BSP_IOEXP_DEVICE_ID == BSP_IOExpGetDeviceId()) {
    BSP_PeripheralAccess(BSP_IOEXP_VCOM, 0); // Disable VCOM
    BSP_PeripheralAccess(BSP_IOEXP_DISPLAY, 0); // Disables the display by pulling DISP_ENABLE high.
    BSP_PeripheralAccess(BSP_IOEXP_SENSORS, 1); // Enables the Si7021 sensor on the Wireless STK by pulling SENSOR_ENABLE high
    BSP_PeripheralAccess(BSP_IOEXP_LEDS, 0);  // The LEDs follow the bits LED0 and LED1 when this bit is set
  }
#endif /* FEATURE_IOEXPANDER */

  //Starting the peripherals
  cmu_init();
  LETIMER0_setup();
  TIMER0_setup();
  ADC0_setup();
  ADC0_start();
  SPI_setup();
  I2C_TempSensor_setup();
  LETIMER0_start();

  int8_t rssi;
  uint8_t startTempFlag = 0;
  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* Check for stack event. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {

      case gecko_evt_system_boot_id:
        gecko_cmd_le_gap_set_adv_parameters(LE_MIN_ADVERTISING_INTERVAL_COUNT, LE_MAX_ADVERTISING_INTERVAL_COUNT, 7);
        /* Enter to DFU OTA mode if needed */
		if (boot_to_dfu) {
			gecko_cmd_system_reset(2);
		}

        /* Start general advertising and enable connections. */
        gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);

        struct gecko_msg_flash_ps_load_rsp_t *flash_ps_rsp_t =	gecko_cmd_flash_ps_load(FLASH_TEMP_DATA_KEY);
        if(flash_ps_rsp_t->result == 0)
        {
        	for(int i = 0; i< FLASH_TEMP_DATA_ARRAY_LEN ; i++)
        	{
        		flash_data_temp_set.value_array[i] = flash_ps_rsp_t->value.data[i];
        	}
        	g_temperatureThreshold = flash_data_temp_set.value_float;

        }
        else
        {
			flash_data_temp_set.value_float = g_temperatureThreshold;
			gecko_cmd_flash_ps_save(FLASH_TEMP_DATA_KEY,FLASH_TEMP_DATA_ARRAY_LEN,flash_data_temp_set.value_array);
        }

        break;

      case gecko_evt_le_connection_opened_id:

    	  gecko_cmd_le_connection_set_parameters(evt->data.evt_le_connection_opened.connection, \
    			  	  	  	  	  	  	  	  	  LE_MIN_CONNECTION_INTERVAL_COUNT, \
												  LE_MAX_CONNECTION_INTERVAL_COUNT, \
												  LE_SLAVE_LATENCY, \
												  LE_CONNECTION_TIMEOUT_MS);

    	  break;

      case gecko_evt_gatt_server_characteristic_status_id:
    	  if (evt-> data.evt_gatt_server_characteristic_status.status_flags == gatt_server_confirmation)
    	  {
    		  gecko_cmd_le_connection_get_rssi(evt-> data.evt_gatt_server_characteristic_status.connection);
    	  }
        break;

      case gecko_evt_le_connection_rssi_id:

			rssi = evt->data.evt_le_connection_rssi.rssi;
			if(rssi > -35)
				gecko_cmd_system_set_tx_power(LE_TX_MIN);
			else if(rssi > -45)
				gecko_cmd_system_set_tx_power(-200);
			else if(rssi > -55)
				gecko_cmd_system_set_tx_power(-150);
			else if(rssi > -65)
				gecko_cmd_system_set_tx_power(-50);
			else if(rssi > -75)
				gecko_cmd_system_set_tx_power(0);
			else if(rssi > -85)
				gecko_cmd_system_set_tx_power(50);
			else
				gecko_cmd_system_set_tx_power(LE_TX_MAX);
			break;

	 /* Value of attribute changed from the local database by remote GATT client */
	 case gecko_evt_gatt_server_attribute_value_id:
		/* Check if changed characteristic is the Immediate Alert level */
		if ( gattdb_start_temp_cp == evt->data.evt_gatt_server_attribute_value.attribute)
		{
			if(evt->data.evt_gatt_server_attribute_value.value.data[0] == 0)
			{
				GPIO_PinOutClear(LED0_port,LED0_pin); //remove this. used to debug
				I2C_TempSensor_Sleep();
				startTempFlag = 0;
			}
			else
			{
				GPIO_PinOutSet(LED0_port,LED0_pin); //remove this. used to debug
				I2C_TempSensor_WakeUp();
				startTempFlag = 1;
			}
		}
	  break;

      case gecko_evt_system_external_signal_id:

			if((evt->data.evt_system_external_signal.extsignals & LETIMER0_EXT_EVT_COMP) != 0)
			{

				if(I2C_TempSensor_isEnable() || startTempFlag)
				{
					temperatureMeasureAndSend();
				}
				__disable_irq();
				g_temp_ext_event_status |= LETIMER0_EXT_EVT_COMP;
				__enable_irq();
			}

			if (((evt->data.evt_system_external_signal.extsignals) & BMA_EXT_EVT_TAP) !=0)
			{
				__disable_irq();
				uint8_t tapData = SPI_readByteData(TAP_SENSE_STATE_REGISTER);
				if((DOUBLE_TAP_SENSED & tapData))
				{
					//GPIO_PinOutClear(LED0_port,LED0_pin); //remove this. used to debug
					I2C_TempSensor_Sleep();
				}
				//LED/TEMP SENSOR is off. So now assign the double tap INT to INT1 to turn on  the LED/TEMP SENSOR
				else if((SINGLE_TAP_SENSED & tapData))
				{
					//GPIO_PinOutSet(LED0_port,LED0_pin); //remove this. used to debug
					I2C_TempSensor_WakeUp();
				}

				g_temp_ext_event_status |= BMA_EXT_EVT_TAP;
				__enable_irq();
			}

			if (((evt->data.evt_system_external_signal.extsignals) & ADC_EXT_EVT_NORTH) !=0)
			{
				//GPIO_PinOutSet(LED0_port,LED0_pin); //remove this. used to debug
				SPI_start(true);
				//GPIO_PinOutClear(LED0_port,LED0_pin); //remove this. used to debug
				ADC_window_out();
				debounceFlag = true;
				__disable_irq();
				g_temp_ext_event_status |= ADC_EXT_EVT_NORTH;
				__enable_irq();
			}
			if (((evt->data.evt_system_external_signal.extsignals) & ADC_EXT_EVT_SOUTH) !=0)
			{
				//GPIO_PinOutSet(LED0_port,LED0_pin); //remove this. used to debug
				SPI_start(false);
				//GPIO_PinOutClear(LED0_port,LED0_pin); //remove this. used to debug
				ADC_window_out();
				debounceFlag = true;
				__disable_irq();
				g_temp_ext_event_status |= ADC_EXT_EVT_SOUTH;
				__enable_irq();
			}
			if (((evt->data.evt_system_external_signal.extsignals) & ADC_EXT_EVT_WEST) !=0)
			{
				g_temperatureThreshold -= 5;
				ADC_window_out();
				debounceFlag = true;
				flash_data_temp_set.value_float = g_temperatureThreshold;
				gecko_cmd_flash_ps_save(FLASH_TEMP_DATA_KEY,FLASH_TEMP_DATA_ARRAY_LEN,flash_data_temp_set.value_array);
				__disable_irq();
				g_temp_ext_event_status |= ADC_EXT_EVT_WEST;
				__enable_irq();
			}
			if (((evt->data.evt_system_external_signal.extsignals) & ADC_EXT_EVT_EAST) !=0)
			{
				g_temperatureThreshold += 5;
				ADC_window_out();
				debounceFlag = true;
				flash_data_temp_set.value_float = g_temperatureThreshold;
				gecko_cmd_flash_ps_save(FLASH_TEMP_DATA_KEY,FLASH_TEMP_DATA_ARRAY_LEN,flash_data_temp_set.value_array);
				__disable_irq();
				g_temp_ext_event_status |= ADC_EXT_EVT_EAST;
				__enable_irq();
			}
			if (((evt->data.evt_system_external_signal.extsignals) & ADC_EXT_EVT_CENTER) !=0)
			{
				GPIO_PinOutClear(LED1_port,LED1_pin);
				ADC_window_out();
				debounceFlag = true;
				__disable_irq();
				g_temp_ext_event_status |= ADC_EXT_EVT_CENTER;
				__enable_irq();
			}

			__disable_irq();
			g_external_event_status &= ~g_temp_ext_event_status;
			__enable_irq();
    	  break;

      case gecko_evt_le_connection_closed_id:
        /* Check if need to boot to dfu mode */
        if (boot_to_dfu) {
          /* Enter to DFU OTA mode */
          gecko_cmd_system_reset(2);
        } else {
          gecko_cmd_system_set_tx_power(0);
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
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
          gecko_cmd_endpoint_close(evt->data.evt_gatt_server_user_write_request.connection);
        }
        break;

      default:
        break;
    }
  }
  return 0;
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
