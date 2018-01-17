/*
 * sleep.c
 *
 *  Created on: 12-Sep-2017
 *      Author: Gunj Manseta
 */
#include <sleep_g.h>
#include "em_emu.h"
#include <stdbool.h>

int sleepStateBlockCounter[5] = {0,0,0,0,0};

//***********************************************************************************
// functions definition

/***********************************************************************************
*******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/
/* Sleep routine is the property of the Silicon Labs.
 * Assume this to be any due credit given to the owner of the routine.
 * Sleep routine is used to for energy efficiency.
 * This checks the sleepStateBlockCounter global variable and goes to specific allowed energy state to
 * reduce power consumption.
 * Note: Avoiding EM4 sleep state as there is a risk of Bricking the device for now. Will change it afterwards.
 * This step was suggested in the class by Prof.
 */
void sleep()
{
//	if(sleepStateBlockCounter[EM0] > 0)
//		return;
//	else if(sleepStateBlockCounter[EM1] > 0)
//		EMU_EnterEM1();
//	else if(sleepStateBlockCounter[EM2] > 0)
//		EMU_EnterEM2(true);
//	else //if(sleepStateBlockCounter[EM3]>0)
//		EMU_EnterEM3(true);
//	//else
//		//EMU_EnterEM4();
}

/*Block the board from going to sleep below a certain energy state
 * This changes the global variable which keeps a counter of peripherals using the specific energy state
 * Hence, it wont allow the sleep routine to enter a specific energy mode
 */
void blockSleepMode(sleepEnergyStateEnum minimumMode)
{
//	__disable_irq();
//	//CORE_AtomicDisableIrq();
//	sleepStateBlockCounter[minimumMode]++;
//	__enable_irq();
//	//CORE_AtomicEnableIrq();
}
/*Unblock the board from going to sleep below a certain energy state
 * This changes the global variable which keeps a counter of peripherals using the specific energy state
 * Hence, once called, it will allow the sleep routine to enter a specific energy mode provided there are no other
 * peripherals blocking the particular energy state.
 */
void unblockSleepMode(sleepEnergyStateEnum minimumMode)
{
//	__disable_irq();
//	//CORE_AtomicDisableIrq();
//	if(sleepStateBlockCounter[minimumMode] > 0)
//		sleepStateBlockCounter[minimumMode]--;
//	//CORE_AtomicEnableIrq();
//	__enable_irq();
}
