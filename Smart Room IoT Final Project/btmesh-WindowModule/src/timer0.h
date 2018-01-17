/*
 * timer0.h
 *
 *  Created on: 26-Sep-2017
 *      Author: Gunj Manseta
 */

#ifndef SRC_TIMER0_H_
#define SRC_TIMER0_H_

#define ENERGY_MODE_TIMER (EM1)
volatile bool TIMER_INT_SERVED;
//***********************************************************************************
// function prototypes
//***********************************************************************************

void TIMER0_setup();
void TIMER0_startwithCount(uint32_t timerCount);

void TIMER0_IRQHandler();


#endif /* SRC_TIMER0_H_ */
