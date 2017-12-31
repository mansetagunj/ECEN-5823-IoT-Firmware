/*
 * adc0.h
 *
 *  Created on: 19-Sep-2017
 *      Author: Gunj Manseta
 */

#ifndef SRC_ADC0_H_
#define SRC_ADC0_H_

#include <stdbool.h>

#define ENERGY_MODE_ADC0 	(EM3)
#define ADC_EXT_EVT_NORTH	(0x08)
#define ADC_EXT_EVT_SOUTH	(0x10)
#define ADC_EXT_EVT_WEST	(0x20)
#define ADC_EXT_EVT_CENTER	(0x40)
#define ADC_EXT_EVT_EAST	(0x80)

volatile bool debounceFlag;

//***********************************************************************************
// function prototypes
//***********************************************************************************
void ADC0_setup(void);
void ADC0_start(void);
void ADC_window_out();
void ADC_window_in();


#endif /* SRC_ADC0_H_ */
