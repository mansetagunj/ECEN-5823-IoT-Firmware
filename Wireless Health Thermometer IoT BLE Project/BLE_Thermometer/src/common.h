/*
 * common.h
 *
 *  Created on: 13-Sep-2017
 *      Author: Gunj Manseta
 */

#ifndef SRC_COMMON_H_
#define SRC_COMMON_H_

#include "sleep_g.h"

/* *********************INTERRUPT PRIORITY TABLE **************
 *
 * Lower the priority number, higher is the priority.
 * As the I2C0 operation is called within the
 * LETIMER0 IRQHandler, I2C0 has higher priority
 * -------------------------------------------------
 * Priority - I2C0_IRQn 	-> 	2
 * Priority - LETIMER0_IRQn ->	3
 *--------------------------------------------------
 *--------------------------------------------------
 * SPI operation is called from the ADC0 IRQHander,
 * which further starts the TIMER0 and waits for the
 * flag which is set in the TIMER0 IRQHandler.
 * So, effectively, TIMER0 is interrupted from the
 * ADC0 IRQHandler. Hence, TIMER0 has higher priority
 * -------------------------------------------------
 * Priority - TIMER0_IRQn 	->	4
 * Priority - ADC0_IRQn 	->	5
 * Priority - GPIO_ODD_IRQn ->	6
 * -------------------------------------------------
 *
 **************************************************************/

#define IRQ_PRIORITY_I2C0 (2)
#define IRQ_PRIORITY_LETIMER0 (3)
#define IRQ_PRIORITY_TIMER0 (4)
#define IRQ_PRIORITY_ADC0 (5)


#define LETIMER_MIN_ENERGYSTATE (EM3)

#define LFXO_CLOCK_FREQUENCY (32768)
#define ULFRCO_CLOCK_FREQUENCY (1000)

#define PERIOD_CYCLE_MILLISECONDS (1750)
#define ON_DUTY_CYCLE_MILLISECONDS (20)
#define CHANGE_IN_DUTY_CYCLE_MILLISECONDS (500)

#define LE_MIN_ADVERTISING_INTERVAL_MS		(500)
#define LE_MAX_ADVERTISING_INTERVAL_MS		(500)
#define LE_MIN_ADVERTISING_INTERVAL_COUNT	(800)			//count = LE_MIN_ADVERTISING_INTERVAL_MS * 1.6
#define LE_MAX_ADVERTISING_INTERVAL_COUNT	(800)			//count = LE_MAX_ADVERTISING_INTERVAL_MS * 1.6
#define LE_MIN_CONNECTION_INTERVAL_MS		(75)
#define LE_MAX_CONNECTION_INTERVAL_MS		(75)
#define LE_MIN_CONNECTION_INTERVAL_COUNT	(60)			//Count = LE_MIN_CONNECTION_INTERVAL_MS/1.25
#define LE_MAX_CONNECTION_INTERVAL_COUNT	(60)			//Count = LE_MAX_CONNECTION_INTERVAL_MS/1.25
#define LE_SLAVE_LATENCY_MS					(375)
#define LE_SLAVE_LATENCY					(4)				//Latency = (LE_SLAVE_LATENCY_MS/LE_MAX_CONNECTION_INTERVAL_MS) - 1
#define LE_CONNECTION_TIMEOUT_MS			(600)			//Timeout >= (latency_ms*2)
#define LE_TX_MAX							(80)
#define LE_TX_MIN							(-260)

//volatile int16_t g_temperatureThreshold;
volatile float g_temperatureThreshold;
extern volatile uint32_t g_external_event_status;


#endif /* SRC_COMMON_H_ */
