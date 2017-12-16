/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Bluetooth mesh light switch example
 *
 * This example implements a Bluetooth mesh light switch.
 *
 ***************************************************************************************************
 * <b> (C) Copyright 2017 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 * --------------------------------------------------------------------------------------------------------
 * The file has been modified by Gunj Manseta - University of Colorado Boulder
 * This file is modified in order to experiment and build a concept.
 * This was the part of a project at University of Colorado Boulder for the wonderful
 * subject ECEN 5823 IoT Embedded Firmware taught by Prof. Keith Graham where we wanted to
 * experiment on the Bluetooth Mesh technology.
 * For more information on the project, visit https://github.com/mansetagunj.
 * Credits to Silicon Labs for this wonderful example code on Bluetooth Mesh
 * --------------------------------------------------------------------------------------------------------
 *
 **************************************************************************************************/

/* C Standard Library headers */
#include <stdlib.h>
#include <stdio.h>

/* WSTK specific includes */
#include "boards.h"
#include "btMesh-configuration.h"
#include "board_features.h"
#include "retargetserial.h"
#include "graphics.h"

/* Silicon Labs radio board specific includes */
#ifdef FEATURE_BOARD_DETECTED
#include "bspconfig.h"
#include "pti.h"
#endif

/* Included if the Silicon Labs radio board has a SPI flash */
#ifdef FEATURE_SPI_FLASH
#include "em_usart.h"
#include "mx25flash_spi.h"
#endif /* FEATURE_SPI_FLASH */

/* emLib and emDrv (HW drivers) specific includes */
#include <em_gpio.h>
#include <gpiointerrupt.h>

/* Bluetooth LE stack includes */
#include <native_gecko.h>
#include <gecko_configuration.h>

/* Bluetooth LE GATT database */
#include "gatt_db.h"

/* Bluetooth mesh stack includes */
#include "gecko_bgapi_mesh_node_native.h"
#include "mesh_generic_model_capi_types.h"
#include "gecko_bgapi_mesh_generic_server_native.h"
#include "gecko_bgapi_mesh_generic_client_native.h"
#include "mesh_lib.h"

/* EFR32 hardware initialization */
#include "InitDevice.h"

//custom headers
#include "adc0.h"

struct mesh_generic_state current, target;

// Maximum number of simultaneous Bluetooth connections
#define MAX_CONNECTIONS 2

// heap for Bluetooth stack
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + 8000];

/*
 * Maximum number of Bluetooth advertisement sets.
 * 1 is allocated for Bluetooth LE stack
 * 1 one for Bluetooth mesh stack
 * 1 needs to be allocated for each Bluetooth mesh network
 *   - Currently up to 4 networks are supported at a time
 */
#define MAX_ADVERTISERS (2 + 4)

// Bluetooth stack configuration
const gecko_configuration_t config =
{
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.max_advertisers = MAX_ADVERTISERS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100,
  .gattdb = &bg_gattdb_data,
};

/** Timer Frequency used. */
#define TIMER_CLK_FREQ ((uint32)32768)
/** Convert msec to timer ticks. */
#define TIMER_MS_2_TIMERTICK(ms) ((TIMER_CLK_FREQ * (ms)) / 1000)

#define TIMER_ID_RESTART    78
#define TIMER_ID_FACTORY_RESET  77
#define TIMER_ID_PROVISIONING   66
#define TIMER_ID_RETRANS    10
#define TIMER_ID_TRANSITION   55
#define TIMER_ID_OOB_CLEAR   25
#define TIMER_ID_DOOR_LOCK   35

/**
 *  LCD content can be updated one row at a time using function LCD_write().
 *  Row number is passed as parameter,the possible values are defined below.
 */
#define LCD_ROW_HEADER     		1  	 /* 1st row, device sub header */
#define LCD_ROW_NAME     		2  	 /* 2st row, device name */
#define LCD_ROW_STATUS     		3    /* 3nd row, node status */
#define LCD_ROW_CONNECTION 		4    /* 4rd row, connection status */
#define LCD_ROW_DATA	 		5    /* 5rd row, data */
#define LCD_ROW_MODULE_STATE 	6    /* 6th row, module state */
#define LCD_ROW_MODULE_OOB_NUM 	7    /* 7th row, OOB rand number */
#define LCD_ROW_MAX        		7    /* total number of rows used */

#define LCD_ROW_LEN        32   /* up to 32 characters per each row */

/** global variables */
static uint16 _my_index = 0xffff; /* Index of the Primary Element of the Node */
static uint16 _my_address = 0;    /* Address of the Primary Element of the Node */
static uint8 switch_pos = 0;      /* current position of the switch  */
static uint8 request_count;       /* number of on/off requests to be sent */
static uint8 trid = 0;        /* transaction identifier */
static char LCD_data[LCD_ROW_MAX][LCD_ROW_LEN];   /* 2D array for storing the LCD content */
static uint8 num_connections = 0;     /* number of active Bluetooth connections */
static uint8 conn_handle = 0xFF;      /* handle of the last opened LE connection */
static char *MODULE_NAME = "DOOR MODULE";
static uint8 door_locked_onApp = 0;

#define PRESS_NORTH_KEY 	1	//North - 1
#define PRESS_SOUTH_KEY 	2	//South - 2
#define PRESS_WEST_KEY 		3	//West  - 3
#define PRESS_EAST_KEY 		4	//East - 4
#define KEY_LENGTH			4
const uint8 door_unlock_key[KEY_LENGTH] = {PRESS_SOUTH_KEY,PRESS_WEST_KEY,PRESS_NORTH_KEY,PRESS_EAST_KEY};	//The hardcoded key that needs to be replicated by the user
//the key provided by the user should be filled in door_unlock_user_key array
uint8 door_unlock_user_key[KEY_LENGTH] = {0};
uint8 door_unlock_index = 0;
bool door_pattern_matched = 0;
volatile uint32_t g_external_event_status = 0;
volatile uint32_t g_temp_ext_event_status = 0;
char dumpString[4] = {0};

#define DOOR_UNLOCK_PERIOD	(10000)	//in mSeconds
#define OOB_DISPLAY_PERIOD	(15000)	//in mSeconds

/**
 *  State of the LEDs is updated by calling LED_set_state().
 *  The new state is passed as parameter, possible values are defined below.
 */
#define LED_STATE_OFF    0   /* light off (both LEDs turned off)   */
#define LED_STATE_ON     1   /* light on (both LEDs turned on)     */
#define LED_STATE_PROV   3   /* provisioning (LEDs blinking)       */

/**
 *  These are needed to support radio boards with active-low and
 *  active-high LED configuration
 */
#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
/* LED GPIO is active-low */
#define TURN_LED_OFF   GPIO_PinOutSet
#define TURN_LED_ON    GPIO_PinOutClear
#define LED_DEFAULT_STATE  1
#else
/* LED GPIO is active-high */
#define TURN_LED_OFF   GPIO_PinOutClear
#define TURN_LED_ON    GPIO_PinOutSet
#define LED_DEFAULT_STATE  0
#endif

/**
 *  State of the LEDs is updated by calling LED_set_state().
 *  The new state is passed as parameter, possible values are defined below.
 */
#define LED_STATE_OFF    0   /* light off (both LEDs turned off)   */
#define LED_STATE_ON     1   /* light on (both LEDs turned on)     */
#define LED_STATE_TRANS  2   /* transition (LED0 on, LED1 off)     */
#define LED_STATE_PROV   3   /* provisioning (LEDs blinking)       */

static struct lightbulb_state {
  // On/Off Server state
  uint8_t onoff_current;
  uint8_t onoff_target;

  // Transition Time Server state
  uint8_t transtime;

  // On Power Up Server state
  uint8_t onpowerup;
} lightbulb_state;

/**
 * Update the state of LEDs. Takes one parameter LED_STATE_xxx that defines
 * the new state.
 */
static void LED_set_state(int state)
{
  switch (state) {
    case LED_STATE_OFF:
      TURN_LED_OFF(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
      TURN_LED_OFF(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
      break;
    case LED_STATE_ON:
      TURN_LED_ON(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
      TURN_LED_ON(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
      break;
    case LED_STATE_PROV:
      GPIO_PinOutToggle(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
      GPIO_PinOutToggle(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
      break;

    default:
      break;
  }
}

static int lightbulb_state_load(void);
static int lightbulb_state_store(void);

static uint32_t default_transition_time(void)
{
  return mesh_lib_transition_time_to_ms(lightbulb_state.transtime);
}


static errorcode_t onoff_response(uint16_t element_index,
                                  uint16_t client_addr,
                                  uint16_t appkey_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_generic_state_on_off;
  current.on_off.on = lightbulb_state.onoff_current;

  target.kind = mesh_generic_state_on_off;
  target.on_off.on = lightbulb_state.onoff_target;

  return mesh_lib_generic_server_response(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                          element_index,
                                          client_addr,
                                          appkey_index,
                                          &current,
                                          NULL,
                                          0,
                                          0x00);
}

static errorcode_t onoff_update(uint16_t element_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_generic_state_on_off;
  current.on_off.on = lightbulb_state.onoff_current;

  target.kind = mesh_generic_state_on_off;
  target.on_off.on = lightbulb_state.onoff_target;

  return mesh_lib_generic_server_update(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        NULL,
                                        0);
}

static errorcode_t onoff_update_and_publish(uint16_t element_index)
{
  errorcode_t e;

  e = onoff_update(element_index);
  if (e == bg_err_success) {
    e = mesh_lib_generic_server_publish(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        mesh_generic_state_on_off);
  }

  return e;
}

static void onoff_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags)
{
  printf("ON/OFF request: requested state=<%s>, transition=%u, delay=%u\r\n",
         request->on_off ? "ON" : "OFF", transition_ms, delay_ms);
  printf("Client Addr: %x, Server Addr: %x, Appkey Index: %d\r\n",client_addr,server_addr,appkey_index);
  if (lightbulb_state.onoff_current == request->on_off) {
    printf("Request for current state received; no op\n");
  } else {
    printf("Turning lightbulb <%s>\r\n", request->on_off ? "ON" : "OFF");
    if (transition_ms == 0 && delay_ms == 0) { // Immediate change
      lightbulb_state.onoff_current = request->on_off;
      lightbulb_state.onoff_target = request->on_off;
      if (lightbulb_state.onoff_current == MESH_GENERIC_ON_OFF_STATE_OFF) {
        LED_set_state(LED_STATE_OFF);
        //LCD_write("STATE - OFF", LCD_ROW_MODULE_STATE);
        LCD_write("Door Locked", LCD_ROW_MODULE_STATE);
      } else {
        LED_set_state(LED_STATE_ON);
        //LCD_write("STATE - ON", LCD_ROW_MODULE_STATE);
        LCD_write("Door Unlocked", LCD_ROW_MODULE_STATE);
      }
    } else {
      // Current state remains as is for now
      lightbulb_state.onoff_target = request->on_off;
      LED_set_state(LED_STATE_TRANS); // set LEDs to transition mode
      LCD_write("State is changing", LCD_ROW_MODULE_STATE);
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(delay_ms + transition_ms), TIMER_ID_TRANSITION, 1);
      printf("Delay count: %lu",TIMER_MS_2_TIMERTICK(delay_ms + transition_ms));
    }
    lightbulb_state_store();
    lightbulb_state.onpowerup = MESH_GENERIC_ON_POWER_UP_STATE_OFF;	//gunj
  }

  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
    onoff_response(element_index, client_addr, appkey_index);
  } else {
    onoff_update(element_index);
  }
}

static void onoff_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms)
{
  // TODO
}

static errorcode_t power_onoff_response(uint16_t element_index,
                                        uint16_t client_addr,
                                        uint16_t appkey_index)
{
  struct mesh_generic_state current;
  current.kind = mesh_generic_state_on_power_up;
  current.on_power_up.on_power_up = lightbulb_state.onpowerup;

  return mesh_lib_generic_server_response(MESH_GENERIC_POWER_ON_OFF_SERVER_MODEL_ID,
                                          element_index,
                                          client_addr,
                                          appkey_index,
                                          &current,
                                          NULL,
                                          0,
                                          0x00);
}

static errorcode_t power_onoff_update(uint16_t element_index)
{
  struct mesh_generic_state current;
  current.kind = mesh_generic_state_on_power_up;
  current.on_power_up.on_power_up = lightbulb_state.onpowerup;

  return mesh_lib_generic_server_update(MESH_GENERIC_POWER_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        NULL,
                                        0);
}

static errorcode_t power_onoff_update_and_publish(uint16_t element_index)
{
  errorcode_t e;

  e = power_onoff_update(element_index);
  if (e == bg_err_success) {
    e = mesh_lib_generic_server_publish(MESH_GENERIC_POWER_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        mesh_generic_state_on_power_up);
  }

  return e;
}

static void power_onoff_request(uint16_t model_id,
                                uint16_t element_index,
                                uint16_t client_addr,
                                uint16_t server_addr,
                                uint16_t appkey_index,
                                const struct mesh_generic_request *request,
                                uint32_t transition_ms,
                                uint16_t delay_ms,
                                uint8_t request_flags)
{
  printf("ON POWER UP request received; state=<%s>\n",
         lightbulb_state.onpowerup == 0 ? "OFF"
         : lightbulb_state.onpowerup == 1 ? "ON"
         : "RESTORE");

  if (lightbulb_state.onpowerup == request->on_power_up) {
    printf("Request for current state received; no op\n");
  } else {
    printf("Setting onpowerup to <%s>\n",
           request->on_power_up == 0 ? "OFF"
           : request->on_power_up == 1 ? "ON"
           : "RESTORE");
    lightbulb_state.onpowerup = request->on_power_up;
    lightbulb_state_store();
  }

  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
    power_onoff_response(element_index, client_addr, appkey_index);
  } else {
    power_onoff_update(element_index);
  }
}

static void power_onoff_change(uint16_t model_id,
                               uint16_t element_index,
                               const struct mesh_generic_state *current,
                               const struct mesh_generic_state *target,
                               uint32_t remaining_ms)
{
  // TODO
}

/**
 * This function is called when a light on/off transition has completed
 */
void transition_complete()
{
  // transition done -> set state, update and publish
  lightbulb_state.onoff_current = lightbulb_state.onoff_target;

  printf("transition complete. New state is %s\r\n", lightbulb_state.onoff_current ? "ON" : "OFF");

  if (lightbulb_state.onoff_current == MESH_GENERIC_ON_OFF_STATE_OFF) {
    LED_set_state(LED_STATE_OFF);
    //LCD_write("STATE - OFF", LCD_ROW_MODULE_STATE);
    LCD_write("Door Locked", LCD_ROW_MODULE_STATE);
  } else {
    LED_set_state(LED_STATE_ON);
    //LCD_write("STATE - ON", LCD_ROW_MODULE_STATE);
    LCD_write("Door Unlocked", LCD_ROW_MODULE_STATE);
  }

  lightbulb_state_store();
  onoff_update_and_publish(_my_index);
}


static errorcode_t transtime_response(uint16_t element_index,
                                      uint16_t client_addr,
                                      uint16_t appkey_index)
{
  struct mesh_generic_state current;
  current.kind = mesh_generic_state_transition_time;
  current.transition_time.time = lightbulb_state.transtime;

  return mesh_lib_generic_server_response(MESH_GENERIC_TRANSITION_TIME_SERVER_MODEL_ID,
                                          element_index,
                                          client_addr,
                                          appkey_index,
                                          &current,
                                          NULL,
                                          0,
                                          0x00);
}

static errorcode_t transtime_update(uint16_t element_index)
{
  struct mesh_generic_state current;
  current.kind = mesh_generic_state_transition_time;
  current.transition_time.time = lightbulb_state.transtime;

  return mesh_lib_generic_server_update(MESH_GENERIC_TRANSITION_TIME_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        NULL,
                                        0);
}

static void transtime_request(uint16_t model_id,
                              uint16_t element_index,
                              uint16_t client_addr,
                              uint16_t server_addr,
                              uint16_t appkey_index,
                              const struct mesh_generic_request *request,
                              uint32_t transition_ms,
                              uint16_t delay_ms,
                              uint8_t request_flags)
{
  printf("TRANSTIME request received; state=<0x%x>\n",
         lightbulb_state.transtime);

  if (lightbulb_state.transtime == request->transition_time) {
    printf("Request for current state received; no op\n");
  } else {
    printf("Setting transtime to <0x%x>\n", request->transition_time);
    lightbulb_state.transtime = request->transition_time;
    lightbulb_state_store();
  }

  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
    transtime_response(element_index, client_addr, appkey_index);
  } else {
    transtime_update(element_index);
  }
}

static void transtime_change(uint16_t model_id,
                             uint16_t element_index,
                             const struct mesh_generic_state *current,
                             const struct mesh_generic_state *target,
                             uint32_t remaining_ms)
{
  // TODO
}

/**
 * This function loads the saved light state from Persistent Storage and
 * copies the data in the global variable lightbulb_state
 */
static int lightbulb_state_load(void)
{
  struct gecko_msg_flash_ps_load_rsp_t* pLoad;

  pLoad = gecko_cmd_flash_ps_load(0x4004);

  if (pLoad->result) {
    memset(&lightbulb_state, 0, sizeof(struct lightbulb_state));
    return -1;
  }

  memcpy(&lightbulb_state, pLoad->value.data, pLoad->value.len);

  return 0;
}

/**
 * this function saves the current light state in Persistent Storage so that
 * the data is preserved over reboots and power cycles. The light state is hold
 * in a global variable lightbulb_state. a PS key with ID 0x4004 is used to store
 * the whole struct.
 */
static int lightbulb_state_store(void)
{
  struct gecko_msg_flash_ps_save_rsp_t* pSave;

  pSave = gecko_cmd_flash_ps_save(0x4004, sizeof(struct lightbulb_state), (const uint8*)&lightbulb_state);

  if (pSave->result) {
    printf("lightbulb_state_store(): PS save failed, code %x\r\n", pSave->result);
    return(-1);
  }

  return 0;
}




#if (EMBER_AF_BOARD_TYPE == BRD4304A)
/**
 * LNA init. Required only for MGM12P radio board (BRD4304A)
 */
#define PRS_CH_CTRL_SIGSEL_RACLNAEN 0x00000003UL
#define PRS_CH_CTRL_SIGSEL_RACPAEN 0x00000004UL
#define PRS_CH_CTRL_SOURCESEL_RAC (0x00000051UL << 8)

static void LNA_init(void)
{
  GPIO_PinModeSet(gpioPortD, 10, gpioModePushPullAlternate, 0);
  GPIO_PinModeSet(gpioPortD, 11, gpioModePushPullAlternate, 0);

  PRS->CH[5].CTRL = PRS_CH_CTRL_SIGSEL_RACLNAEN
                    | PRS_CH_CTRL_SOURCESEL_RAC
                    | PRS_CH_CTRL_EDSEL_OFF;

  PRS->CH[6].CTRL = PRS_CH_CTRL_ORPREV
                    | PRS_CH_CTRL_SIGSEL_RACPAEN
                    | PRS_CH_CTRL_SOURCESEL_RAC
                    | PRS_CH_CTRL_EDSEL_OFF;

  PRS->ROUTELOC1 |= PRS_ROUTELOC1_CH5LOC_LOC0;  // loc0 -> PD10
  PRS->ROUTELOC1 |= PRS_ROUTELOC1_CH6LOC_LOC13; // loc12 -> PD11
  PRS->ROUTEPEN |= (PRS_ROUTEPEN_CH5PEN | PRS_ROUTEPEN_CH6PEN);
}
#endif

/**
 * This is a callback function that is invoked each time a GPIO interrupt in one of the pushbutton
 * inputs occurs. Pin number is passed as parameter.
 *
 * Note: this function is called from ISR context and therefore it is not possible to call any BGAPI
 * functions directly. The button state change is signaled to the application using gecko_external_signal()
 * that will generate an event gecko_evt_system_external_signal_id which is then handled in the main loop.
 */
void gpioint(uint8_t pin)
{
  if (pin == BSP_GPIO_PB0_PIN) {
    gecko_external_signal(0x1);
  } else if (pin == BSP_GPIO_PB1_PIN) {
    gecko_external_signal(0x2);
  }
}

/**
 * Enable button interrupts for PB0, PB1. Both GPIOs are configured to trigger an interrupt on the
 * rising edge (button released).
 */
void enable_button_interrupts(void)
{
  GPIOINT_Init();

  GPIO_ExtIntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, BSP_GPIO_PB0_PIN, true, false, true);
  GPIO_ExtIntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, BSP_GPIO_PB1_PIN, true, false, true);

  /* register the callback function that is invoked when interrupt occurs */
  GPIOINT_CallbackRegister(BSP_GPIO_PB0_PIN, gpioint);
  GPIOINT_CallbackRegister(BSP_GPIO_PB1_PIN, gpioint);
}

/**
 * This function publishes one on/off request to change the state of light(s) in the group.
 * Global variable switch_pos holds the latest desired light state, possible values are
 * switch_pos = 1 -> PB1 was pressed, turn lights on
 * switch_pos = 0 -> PB0 was pressed, turn lights off
 *
 * This application sends multiple requests for each button press to improve reliability.
 * Parameter retrans indicates whether this is the first request or a re-transmission.
 * The transaction ID is not incremented in case of a re-transmission.
 */
void send_onoff_request(int retrans)
{
  uint16 resp;
  uint16 delay;
  struct mesh_generic_request req;

  req.kind = mesh_generic_request_on_off;
  req.on_off = switch_pos ? MESH_GENERIC_ON_OFF_STATE_ON : MESH_GENERIC_ON_OFF_STATE_OFF;

  // increment transaction ID for each request, unless it's a retransmission
  if (retrans == 0) {
    trid++;
  }

  /* delay for the request is calculated so that the last request will have a zero delay and each
   * of the previous request have delay that increases in 50 ms steps. For example, when using three
   * on/off requests per button press the delays are set as 100, 50, 0 ms
   */
  delay = (request_count - 1) * 50;
  	resp = gecko_cmd_mesh_generic_client_set(
    MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
    _my_index,
	2,
	0,
    trid,
    0,     // transition
    delay,
    0,     // flags
    mesh_generic_request_on_off,     // type
    1,     // param len
    &req.on_off     /// parameters data
    )->result;


  if (resp) {
    printf("gecko_cmd_mesh_generic_client_publish failed,code %x\r\n", resp);
  } else {
    printf("request sent, trid = %u, delay = %d\r\n", trid, delay);
  }

  /* keep track of how many requests has been sent */
  if (request_count > 0) {
    request_count--;
  }
}

/**
 * Initialization of the models supported by this node. This function registeres callbacks for
 * each of the three supported models.
 */
static void init_models(void)
{
  mesh_lib_generic_server_register_handler(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                           0,
                                           onoff_request,
                                           onoff_change);
  mesh_lib_generic_server_register_handler(MESH_GENERIC_POWER_ON_OFF_SETUP_SERVER_MODEL_ID,
                                           0,
                                           power_onoff_request,
                                           power_onoff_change);
  mesh_lib_generic_server_register_handler(MESH_GENERIC_TRANSITION_TIME_SERVER_MODEL_ID,
                                           0,
                                           transtime_request,
                                           transtime_change);
}

/**
 * Light node initialization. This is called at each boot if provisioning is already done.
 * Otherwise this function is called after provisioning is completed.
 */
void lightbulb_state_init(void)
{
  /* Initialize mesh lib */
  //mesh_lib_init(malloc, free, 8);

  memset(&lightbulb_state, 0, sizeof(struct lightbulb_state));
  if (lightbulb_state_load() != 0) {
    printf("lightbulb_state_load() failed, using defaults\r\n");
    goto publish;
  }

  // Handle on power up behavior
  switch (lightbulb_state.onpowerup) {
    case MESH_GENERIC_ON_POWER_UP_STATE_OFF:
      printf("On power up state is OFF\r\n");
      lightbulb_state.onoff_current = MESH_GENERIC_ON_OFF_STATE_OFF;
      lightbulb_state.onoff_target = MESH_GENERIC_ON_OFF_STATE_OFF;
      LED_set_state(LED_STATE_OFF);
      break;
    case MESH_GENERIC_ON_POWER_UP_STATE_ON:
      printf("On power up state is ON\r\n");
      lightbulb_state.onoff_current = MESH_GENERIC_ON_OFF_STATE_ON;
      lightbulb_state.onoff_target = MESH_GENERIC_ON_OFF_STATE_ON;
      LED_set_state(LED_STATE_ON);
      break;
    case MESH_GENERIC_ON_POWER_UP_STATE_RESTORE:
      printf("On power up state is RESTORE\r\n");
      if (lightbulb_state.onoff_current != lightbulb_state.onoff_target) {
        uint32_t transition_ms = default_transition_time();

        printf("Starting on power up transition\r\n");
        LED_set_state(LED_STATE_TRANS);
        gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(transition_ms), TIMER_ID_TRANSITION, 1);
      } else {
        printf("Keeping loaded state\r\n");
        if (lightbulb_state.onoff_current == MESH_GENERIC_ON_OFF_STATE_OFF) {
          LED_set_state(LED_STATE_OFF);
        } else {
          LED_set_state(LED_STATE_ON);
        }
      }
      break;
  }

  lightbulb_state_store();

  printf("Light bulb state init default\r\n");
  publish:
  init_models();
  onoff_update_and_publish(_my_index);
  power_onoff_update_and_publish(_my_index);
}

/**
 * Handling of button presses. This function called from the main loop when application receives
 * event gecko_evt_system_external_signal_id.
 *
 * parameter button defines which button was pressed, possible values
 * are 0 = PB0, 1 = PB1.
 *
 * This function is called from application context (not ISR) so it is safe to call BGAPI functions
 */
void handle_button_press(int button)
{
  // PB0 -> switch off, PB1 -> switch on
  switch_pos = button;

  printf("PB%d -> turn light(s) ", button);
  if (switch_pos) {
    printf("on\r\n");
  } else {
    printf("off\r\n");
  }

  request_count = 3; // request is sent 3 times to improve reliability

  /* send the first request */
  send_onoff_request(0);

  /* start a repeating soft timer to trigger re-transmission of the request after 50 ms delay */
  gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(50), TIMER_ID_RETRANS, 0);
}

/**
 * Switch node initialization. This is called at each boot if provisioning is already done.
 * Otherwise this function is called after provisioning is completed.
 */
void switch_node_init(void)
{
  mesh_lib_init(malloc, free, 8);
}

static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);

// this is needed by the LCD driver
int rtcIntCallbackRegister(void (*pFunction)(void*),
                           void* argument,
                           unsigned int frequency)
{
  return 0;
}

/**
 * button initialization. Configure pushbuttons PB0,PB1
 * as inputs.
 */
static void button_init()
{
  // configure pushbutton PB0 and PB1 as inputs
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInput, 1);
  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInput, 1);
}

/**
 * LED initialization. Configure LED pins as outputs
 */
static void led_init()
{
  // configure LED0 and LED1 as outputs
  GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull, LED_DEFAULT_STATE);
  GPIO_PinModeSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN, gpioModePushPull, LED_DEFAULT_STATE);
}

/**
 * This function is used to write one line in the LCD. The parameter 'row' selects which line
 * is written, possible values are defined as LCD_ROW_xxx.
 */
void LCD_write(char *str, uint8 row)
{
  char LCD_message[LCD_ROW_MAX * LCD_ROW_LEN];
  char *pRow; /* pointer to selected row */
  int i;

  if (row > LCD_ROW_MAX) {
    return;
  }

  pRow  = &(LCD_data[row - 1][0]);

  sprintf(pRow, str);

  LCD_message[0] = 0;

  for (i = 0; i < LCD_ROW_MAX; i++) {
    pRow  = &(LCD_data[i][0]);
    strcat(LCD_message, pRow);
    strcat(LCD_message, "\n"); // add newline at end of reach row
  }

  graphWriteString(LCD_message);
}

/**
 * LCD initialization, called once at startup.
 */
void LCD_init(void)
{
  /* clear LCD_data table */
  memset(&LCD_data, 0, sizeof(LCD_data));

  /* initialize graphics driver and set the title text */
//  graphInit("SILICON LABORATORIES\nBluetooth Mesh Demo\n\n");
  graphInit("MESH DEMO\nECEN 5823\n");

  LCD_write("initializing", LCD_ROW_STATUS);
}

/**
 * Set device name in the GATT database. A unique name is genrerated using
 * the two last bytes from the Bluetooth address of this device. Name is also
 * displayed on the LCD.
 */
void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16 res;

  // create unique device name using the last two bytes of the Bluetooth address
//  sprintf(name, "switch node %x:%x", pAddr->addr[1], pAddr->addr[0]);
  sprintf(name, "Node %x:%x", pAddr->addr[1], pAddr->addr[0]);

  printf("Device name: '%s - %s'\r\n",MODULE_NAME, name);

  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
  if (res) {
    printf("gecko_cmd_gatt_server_write_attribute_value() failed, code %x\r\n", res);
  }

  // show device name on the LCD
  LCD_write(MODULE_NAME, LCD_ROW_HEADER);
  LCD_write(name, LCD_ROW_NAME);
}

/**
 *  this function is called to initiate factory reset. Factory reset may be initiated
 *  by keeping one of the WSTK pushbuttons pressed during reboot. Factory reset is also
 *  performed if it is requested by the provisioner (event gecko_evt_mesh_node_reset_id)
 */
void initiate_factory_reset(void)
{
  printf("factory reset\r\n");
  LCD_write("\n***\nFACTORY RESET\n***", LCD_ROW_STATUS);

  /* if connection is open then close it before rebooting */
  if (conn_handle != 0xFF) {
    gecko_cmd_endpoint_close(conn_handle);
  }

  /* perform a factory reset by erasing PS storage. This removes all the keys and other settings
     that have been configured for this node */
  gecko_cmd_flash_ps_erase_all();
  // reboot after a small delay
  gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
}

int main()
{
#ifdef FEATURE_SPI_FLASH
  /* Put the SPI flash into Deep Power Down mode for those radio boards where it is available */
  MX25_init();
  MX25_DP();
  /* We must disable SPI communication */
  USART_Reset(USART1);
#endif /* FEATURE_SPI_FLASH */

  enter_DefaultMode_from_RESET();

#if (EMBER_AF_BOARD_TYPE == BRD4304A)
  LNA_init();
#endif

  gecko_init(&config);

#ifdef FEATURE_PTI_SUPPORT
  APP_ConfigEnablePti();
#endif // FEATURE_PTI_SUPPORT

  RETARGET_SerialInit();

  /* initialize LEDs and buttons. Note: some radio boards share the same GPIO for button & LED.
   * Initialization is done in this order so that default configuration will be "button" for those
   * radio boards with shared pins. led_init() is called later as needed to (re)initialize the LEDs
   * */
  led_init();
  button_init();

  LCD_init();
  ADC0_setup();
  ADC0_start();

  while (1) {
    struct gecko_cmd_packet *evt = gecko_wait_event();
    handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
  }
}

/**
 * Handling of stack events. Both BLuetooth LE and Bluetooth mesh events are handled here.
 */
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  struct gecko_bgapi_mesh_node_cmd_packet *node_evt;
  struct gecko_bgapi_mesh_generic_server_cmd_packet *server_evt;
  struct gecko_msg_mesh_node_provisioning_failed_evt_t  *prov_fail_evt;

  if (NULL == evt) {
    return;
  }

  switch (evt_id) {
    case gecko_evt_system_boot_id:
      // check pushbutton state at startup. If either PB0 or PB1 is held down then do factory reset
      if (GPIO_PinInGet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN) == 0 || GPIO_PinInGet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN) == 0) {
        initiate_factory_reset();
      } else {
        struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();

        set_device_name(&pAddr->address);

        // Initialize Mesh stack in Node operation mode, wait for initialized event
        //gecko_cmd_mesh_node_init();
        //Using the OOB_init function for OOB authentication
		gecko_cmd_mesh_node_init_oob(0x01,0x02,0x03,0x06,0x02,0x04,0x01);
		gecko_cmd_system_set_tx_power(-300);

        // re-initialize LEDs (needed for those radio board that share same GPIO for button/LED)
        led_init();
      }
      break;

    case gecko_evt_hardware_soft_timer_id:
      switch (evt->data.evt_hardware_soft_timer.handle) {
        case TIMER_ID_FACTORY_RESET:
          gecko_cmd_system_reset(0);
          break;

        case TIMER_ID_RESTART:
          gecko_cmd_system_reset(0);
          break;

        case TIMER_ID_PROVISIONING:
          LED_set_state(LED_STATE_PROV);
          break;

        case TIMER_ID_RETRANS:
          send_onoff_request(1);   // param 1 indicates that this is a retransmission
          // stop retransmission timer if it was the last attempt
          if (request_count == 0) {
            gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_RETRANS, 0);
          }
          break;

        case TIMER_ID_TRANSITION:
		  /* light state transition has completed */
		  transition_complete();
		  break;

        case TIMER_ID_OOB_CLEAR:
			//clear the OOB number from the LCD
			LCD_write("",LCD_ROW_MODULE_OOB_NUM);
			break;

        case TIMER_ID_DOOR_LOCK:
        	//Locking the door again after 20 seconds
        	lightbulb_state.onoff_current = 0;
			LED_set_state(LED_STATE_OFF);
			LCD_write("Door Locked", LCD_ROW_MODULE_STATE);
			printf("Locking the door back\r\n");
        	break;

        default:
          break;
      }

      break;

    case gecko_evt_mesh_node_initialized_id:
      printf("node initialized\r\n");

      struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

      if (pData->provisioned) {
        printf("node is provisioned. address:%x, ivi:%d\r\n", pData->address, pData->ivi);

        _my_address = pData->address;
        _my_index = 0;   // index of primary element hardcoded to zero in this example

        enable_button_interrupts();
        switch_node_init();
        lightbulb_state_init();
        printf("Light initial state is <%s>\r\n", lightbulb_state.onoff_current ? "ON" : "OFF");
        LCD_write("provisioned", LCD_ROW_STATUS);
        gecko_cmd_system_set_tx_power(-300);
      } else {
        printf("node is unprovisioned\r\n");
        LCD_write("unprovisioned", LCD_ROW_STATUS);

        printf("starting unprovisioned beaconing...\r\n");
        gecko_cmd_mesh_node_start_unprov_beaconing(0x3);   // enable ADV and GATT provisioning bearer
      }
      break;

    case gecko_evt_system_external_signal_id:
    {
      if (evt->data.evt_system_external_signal.extsignals & 0x1) {
      }
      if (evt->data.evt_system_external_signal.extsignals & 0x2) {

      }
      if (((evt->data.evt_system_external_signal.extsignals) & ADC_EXT_EVT_NORTH) !=0)
      {
    	  ADC_window_out();
    	  debounceFlag = true;
    	  //to check is the number of digits entered is not max'd out
    	  if(door_unlock_index < KEY_LENGTH)
    	  {
    		  //storing the user key in the array to be checked on the center press
    		  door_unlock_user_key[door_unlock_index] = PRESS_NORTH_KEY;

    		  //To display the key entered. This used for debug purpose
    		  char temp[2] = {0};
    		  sprintf(temp,"%d",PRESS_NORTH_KEY);
    		  //sprintf(temp,"*");	//for production
    		  strcat(dumpString,temp);
    		  LCD_write(dumpString,LCD_ROW_DATA);

    		  printf("North press. Storing value at index: %d\r\n",door_unlock_index);
    		  door_unlock_index++;
    	  }
    	  //if we have more than the required digits in the key, inform user to press center
    	  else
		  {
			  LCD_write("Press Center", LCD_ROW_DATA);
		  }
		__disable_irq();
		g_temp_ext_event_status |= ADC_EXT_EVT_NORTH;
		__enable_irq();
      }
      if (((evt->data.evt_system_external_signal.extsignals) & ADC_EXT_EVT_SOUTH) !=0)
      {
    	  ADC_window_out();
		  debounceFlag = true;
		  //to check is the number of digits entered is not max'd out
    	  if(door_unlock_index < KEY_LENGTH)
    	  {
    		  //storing the user key in the array to be checked on the center press
    		  door_unlock_user_key[door_unlock_index] = PRESS_SOUTH_KEY;

    		  //To display the key entered. This used for debug purpose
    		  char temp[2] = {0};
    		  sprintf(temp,"%d",PRESS_SOUTH_KEY);
    		  //sprintf(temp,"*");	//for production
    		  strcat(dumpString,temp);
    		  LCD_write(dumpString,LCD_ROW_DATA);

    		  printf("South press. Storing value at index: %d\r\n",door_unlock_index);
    		  door_unlock_index++;
    	  }
    	  //if we have more than the required digits in the key, inform user to press center
    	  else
    	  {
    		  LCD_write("Press Center", LCD_ROW_DATA);
    	  }
		__disable_irq();
		g_temp_ext_event_status |= ADC_EXT_EVT_SOUTH;
		__enable_irq();
      }
      if (((evt->data.evt_system_external_signal.extsignals) & ADC_EXT_EVT_WEST) !=0)
      {
    	  ADC_window_out();
    	  debounceFlag = true;
    	  //to check is the number of digits entered is not max'd out
    	  if(door_unlock_index < KEY_LENGTH)
    	  {
    		  //storing the user key in the array to be checked on the center press
    		  door_unlock_user_key[door_unlock_index] = PRESS_WEST_KEY;

    		  //To display the key entered. This used for debug purpose
    		  char temp[2] = {0};
    		  sprintf(temp,"%d",PRESS_WEST_KEY);
    		  //sprintf(temp,"*");	//for production
    		  strcat(dumpString,temp);
    		  LCD_write(dumpString,LCD_ROW_DATA);

    		  printf("West press. Storing value at index: %d\r\n",door_unlock_index);
    		  door_unlock_index++;
    	  }
    	  //if we have more than the required digits in the key, inform user to press center
    	  else
    	  {
    		  LCD_write("Press Center", LCD_ROW_DATA);
    	  }
		__disable_irq();
		g_temp_ext_event_status |= ADC_EXT_EVT_WEST;
		__enable_irq();
      }
      if (((evt->data.evt_system_external_signal.extsignals) & ADC_EXT_EVT_EAST) !=0)
      {
    	  ADC_window_out();
    	  debounceFlag = true;
    	  //to check is the number of digits entered is not max'd out
    	  if(door_unlock_index < KEY_LENGTH)
    	  {
    		  //storing the user key in the array to be checked on the center press
    		  door_unlock_user_key[door_unlock_index] = PRESS_EAST_KEY;

    		  //To display the key entered. This used for debug purpose
    		  char temp[2] = {0};
    		  sprintf(temp,"%d",PRESS_EAST_KEY);
    		  //sprintf(temp,"*");	//for production
    		  strcat(dumpString,temp);
    		  LCD_write(dumpString,LCD_ROW_DATA);
    		  printf("East press. Storing value at index: %d\r\n",door_unlock_index);
    		  door_unlock_index++;
    	  }
    	  //if we have more than the required digits in the key, inform user to press center
    	  else
    	  {
    		  LCD_write("Press Center", LCD_ROW_DATA);
    	  }
		__disable_irq();
		g_temp_ext_event_status |= ADC_EXT_EVT_EAST;
		__enable_irq();
      }
      if (((evt->data.evt_system_external_signal.extsignals) & ADC_EXT_EVT_CENTER) !=0)
      {
    	  ADC_window_out();
    	  debounceFlag = true;
    	  //to check if we have enough number of presses for the key
    	  if(door_unlock_index < (KEY_LENGTH+1))
    	  {
    		  printf("Center press. Checking pattern for door unlock\r\n");
    		  if(door_unlock_user_key[0] == door_unlock_key[0] &&
    				  door_unlock_user_key[1] == door_unlock_key[1] &&
					  door_unlock_user_key[2] == door_unlock_key[2] &&
					  door_unlock_user_key[3] == door_unlock_key[3] )
    		  {
    			  door_pattern_matched = 1;
    			  door_unlock_index = 0;
    			  printf("Door pattern Matched\r\n");
    			  //to clear the stored correct pattern
    			  memset(door_unlock_user_key,0,KEY_LENGTH);
    			  memset(dumpString,0,KEY_LENGTH);
    			  LCD_write("Pattern Matched", LCD_ROW_MODULE_STATE);
    			  //Clearing the displayed key on the LCD
    			  LCD_write("", LCD_ROW_DATA);
    		  }
    		  else
    		  {
    			  door_pattern_matched = 0;
    			  door_unlock_index = 0;
    			  memset(dumpString,0,KEY_LENGTH);
    			  printf("Door pattern did not matched\r\n");
    			  LCD_write("Wrong Pattern", LCD_ROW_MODULE_STATE);
    			  //Clearing the displayed key on the LCD
    			  LCD_write("", LCD_ROW_DATA);
    		  }
    	  }
    	  //reset the key stored and start from the beginning
    	  else
    	  {
    		  door_unlock_index = 0;
    		  door_pattern_matched = 0;
    		  LCD_write("", LCD_ROW_DATA);
			  //to clear the stored correct pattern
			  memset(door_unlock_user_key,0,KEY_LENGTH);
    		  memset(dumpString,0,KEY_LENGTH);
    		  LCD_write("Enter Pattern", LCD_ROW_MODULE_STATE);
    	  }

		__disable_irq();
		g_temp_ext_event_status |= ADC_EXT_EVT_CENTER;
		__enable_irq();
      }

      __disable_irq();
      g_external_event_status &= ~g_temp_ext_event_status;
      __enable_irq();
    }
    break;

    case gecko_evt_mesh_node_provisioning_started_id:
      printf("Started provisioning\r\n");
      LCD_write("provisioning...", LCD_ROW_STATUS);
#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
      led_init(); /* shared GPIO pins used as LED output */
#endif
      // start timer for blinking LEDs to indicate which node is being provisioned
      gecko_cmd_hardware_set_soft_timer(32768 / 4, TIMER_ID_PROVISIONING, 0);
      break;

    case gecko_evt_mesh_node_provisioned_id:
      _my_index = 0;   // index of primary element hardcoded to zero in this example
      switch_node_init();
      lightbulb_state_init();
      printf("node provisioned, got index=%x\r\n", _my_index);
      // stop LED blinking when provisioning complete
      gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_PROVISIONING, 0);
      LED_set_state(LED_STATE_OFF);
      LCD_write("provisioned", LCD_ROW_STATUS);

#ifdef FEATURE_LED_BUTTON_ON_SAME_PIN
      button_init(); /* shared GPIO pins used as button input */
#endif
      enable_button_interrupts();
      break;

    case gecko_evt_mesh_node_provisioning_failed_id:
      prov_fail_evt = (struct gecko_msg_mesh_node_provisioning_failed_evt_t  *)&(evt->data);
      printf("provisioning failed, code %x\r\n", prov_fail_evt->result);
      LCD_write("prov failed", LCD_ROW_STATUS);
      /* start a one-shot timer that will trigger soft reset after small delay */
      gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_RESTART, 1);
      break;

    case gecko_evt_mesh_generic_server_client_request_id:
      printf("evt gecko_evt_mesh_generic_server_client_request_id\r\n");
	  server_evt = (struct gecko_bgapi_mesh_generic_server_cmd_packet *)evt;
	  //To have a control over the state on the smartphone app and make sure it is in sync with the node state
      if(door_locked_onApp == 0)
      {
    	  //The door pattern matched variable becomes 1 when the user has entered
    	  //the correct 4 digit pattern on the Joystick and a Center press.
    	  if(door_pattern_matched == 1)
    	  {
    		  	  //If the pattern matches,
    		  	  //Change the door state to unlock mode and
    		  	  //further the communication to other Window node
				  mesh_lib_generic_server_event_handler(server_evt);
				  LCD_write("Door Unlocked", LCD_ROW_MODULE_STATE);
				  printf("Unlocking the door\r\n");
				  //handle button press function is used as it is from the SiLabs example program
				  //sending an ON state to the window module
				  handle_button_press(1);
				  gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(DOOR_UNLOCK_PERIOD), TIMER_ID_DOOR_LOCK, 1);
				  door_pattern_matched = 0;
    	  }
    	  else
    	  {
    		  LCD_write("Locked. Pattern?", LCD_ROW_MODULE_STATE);
			  printf("Cannot Unlock the door\r\n");
    	  }
    	  door_locked_onApp = 1;
      }
      else
    	  door_locked_onApp = 0;
      break;

    case gecko_evt_mesh_node_key_added_id:
      node_evt = (struct gecko_bgapi_mesh_node_cmd_packet *)evt;
      printf("got new %s key with index %x\r\n", node_evt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
             node_evt->data.evt_mesh_node_key_added.index);
      break;

    case gecko_evt_mesh_node_model_config_changed_id:
      printf("model config changed\r\n");
      break;

      //This event is used to display the OOB random 6 digit number
    case gecko_evt_mesh_node_display_output_oob_id:
    {
	   struct gecko_msg_mesh_node_display_output_oob_evt_t *pOOB = (struct gecko_msg_mesh_node_display_output_oob_evt_t *)&(evt->data);
	   printf("gecko_msg_mesh_node_display_output_oob_evt_t: action %d, size %d\r\n", pOOB->output_action, pOOB->output_size);
	   for(int i=0;i<pOOB->data.len;i++)
	   {
		   printf("%2.2x ", pOOB->data.data[i]);

	   }
	   printf("\r\n");
	   uint32_t oob_number = pOOB->data.data[pOOB->data.len -1] | (pOOB->data.data[pOOB->data.len -2]<<8) | (pOOB->data.data[pOOB->data.len -3]<<16);
	   char oob_str[13] = {0};
	   sprintf(oob_str,"OTP: %u",oob_number);
	   LCD_write(oob_str,LCD_ROW_MODULE_OOB_NUM);
	   gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(OOB_DISPLAY_PERIOD), TIMER_ID_OOB_CLEAR, 1);
    }
		break;

    case gecko_evt_le_connection_bt5_opened_id:
      printf("evt:gecko_evt_le_connection_bt5_opened_id\r\n");
      num_connections++;
      conn_handle = evt->data.evt_le_connection_bt5_opened.connection;
      LCD_write("connected", LCD_ROW_CONNECTION);
      break;

    case gecko_evt_le_connection_parameters_id:
	  printf("evt:gecko_evt_le_connection_parameters_id\r\n");
	  break;

    case gecko_evt_le_connection_closed_id:
      printf("evt:conn closed, reason 0x%x\r\n", evt->data.evt_le_connection_closed.reason);
      conn_handle = 0xFF;
      if (num_connections > 0) {
        if (--num_connections == 0) {
          LCD_write("", LCD_ROW_CONNECTION);
        }
      }
      break;

    case gecko_evt_mesh_node_reset_id:
      printf("evt gecko_evt_mesh_node_reset_id\r\n");
      initiate_factory_reset();
      break;

    case gecko_evt_le_gap_adv_timeout_id:
    case gecko_evt_le_gap_bt5_adv_timeout_id:
      break;

    default:
      printf("unhandled evt: %x\r\n", evt_id);
      break;
  }
}
