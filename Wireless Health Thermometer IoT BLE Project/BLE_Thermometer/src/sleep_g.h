/*
 * sleep.h
 *
 *  Created on: 12-Sep-2017
 *      Author: Gunj Manseta
 */

#ifndef SRC_SLEEP_G_H_
#define SRC_SLEEP_G_H_


//***********************************************************************************
// global variables
//***********************************************************************************
int sleepStateBlockCounter[5];// = {0,0,0,0,0};

typedef enum sleepEnergyState{
	EM0 = 0,
	EM1,
	EM2,
	EM3,
	EM4
}sleepEnergyStateEnum;


//***********************************************************************************
// function prototypes
//***********************************************************************************
void sleep();
void blockSleepMode(sleepEnergyStateEnum);
void unblockSleepMode(sleepEnergyStateEnum);

#endif /* SRC_SLEEP_G_H_ */
