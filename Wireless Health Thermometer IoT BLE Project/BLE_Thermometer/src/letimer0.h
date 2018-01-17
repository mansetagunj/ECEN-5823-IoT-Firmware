/*
 * letimer0.h
 *
 *  Created on: 13-Sep-2017
 *      Author: Gunj Manseta
 */

#ifndef SRC_LETIMER0_H_
#define SRC_LETIMER0_H_

#include "em_cmu.h"
//***********************************************************************************
// defined files
//***********************************************************************************
//#define LETIMER_MIN_ENERGYSTATE (EM3)
#define LETIMER0_EXT_EVT_COMP (0x01)

//***********************************************************************************
// global variables
//***********************************************************************************
uint32_t g_prescalerValue;
volatile uint32_t g_currentOnDutyCycle_msec;
//***********************************************************************************
// function prototypes
//***********************************************************************************

void LETIMER0_init(void);
void LETIMER0_setup(void);
void LETIMER0_start(void);
CMU_ClkDiv_TypeDef LETIMER0_calculateAndGetPrescaler(uint32_t period_msec);
void LETIMER0_setCOMPValues(uint32_t period_msec, uint32_t onDutyCycle_msec);
void LETIMER0_increaseOnDutyCycle(uint32_t onDutyCycle_msec);
void LETIMER0_decreaseOnDutyCycle(uint32_t onDutyCycle_msec);
#endif /* SRC_LETIMER0_H_ */
