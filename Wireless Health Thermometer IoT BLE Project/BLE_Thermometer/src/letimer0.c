/*
 * letimer0.c
 *
 *  Created on: 13-Sep-2017
 *      Author: Gunj Manseta
 */

#include "em_letimer.h"
#include "em_device.h"
#include "letimer0.h"
#include "cmu.h"
#include "gpio.h"
#include "common.h"
#include "I2C_TempSensor.h"
#include "native_gecko.h"

uint32_t g_prescalerValue = 0;
volatile uint32_t g_currentOnDutyCycle_msec = 0;
//***********************************************************************************
// functions definition
//***********************************************************************************

void LETIMER0_init(void)
{
	/* Disabling the LETIMER0 */
	LETIMER_Enable(LETIMER0, false);

	/* Configuring the LETIMER0	 */
	const LETIMER_Init_TypeDef letimerInit =
	{
	//.enable         = true,                 	/* Start counting when init completed. */
	.enable			= false,					/*Do not start counting when init is completed. Use Enable function instead */
	.debugRun       = false,                  	/* Counter shall not keep running during debug halt. */
	.comp0Top       = true,                   	/* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
	.bufTop         = false,                  	/* Don't load COMP1 into COMP0 when REP0 reaches 0. */
	.out0Pol        = 0,                      	/* Idle value for output 0. */
	.out1Pol        = 0,                      	/* Idle value for output 1. */
	.ufoa0          = letimerUFOANone,         	/* No output action */
	.ufoa1          = letimerUFOANone,       	/* No output action */
	.repMode        = letimerRepeatFree      	/* Count until stopped */
	};

	/* Initialize LETIMER */
	LETIMER_Init(LETIMER0, &letimerInit);
}

void LETIMER0_setup(void)
{
	CMU_ClkDiv_TypeDef prescaler = LETIMER0_calculateAndGetPrescaler(PERIOD_CYCLE_MILLISECONDS);
	/* Initializing the necessary clocks */
	cmu_init_LETIMER0(prescaler);

	/* Setting up the LED0 which will be used as the output */
	gpio_LETIMER0_init();

	/* Function calls for setting up the COMP0 and COMP1 registers
	 * Hard coding the values due to less time in hand.
	 * Change it to a more dynamic approach calculating the value as per the Time period and Duty Cycle required.
	 * */
	LETIMER0_setCOMPValues(PERIOD_CYCLE_MILLISECONDS,ON_DUTY_CYCLE_MILLISECONDS);
	/*if(LETIMER_MIN_ENERGYSTATE < EM3)
	{
		LETIMER_CompareSet(LETIMER0, 0, 40960);
		LETIMER_CompareSet(LETIMER0, 1, 40141); // 40960 (count for 2.5s)  - 819 (count for .05s)
	}
	else //for ULFRCO Freq. is 1000 Hz
	{
		LETIMER_CompareSet(LETIMER0, 0, 2500);
		LETIMER_CompareSet(LETIMER0, 1, 2450); // 2500 (count for 2.5s)  - 50 (count for .05s)
	}*/

	LETIMER0_init();
}

void LETIMER0_start(void)
{
	blockSleepMode(LETIMER_MIN_ENERGYSTATE);

	/* Enable COMP0 and COMP1 match interrupt */
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP0);
	//LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP1);
	//LETIMER_IntDisable(LETIMER0, LETIMER_IF_UF);

	//Refer common.h for INT priority table
	NVIC_SetPriority(LETIMER0_IRQn, IRQ_PRIORITY_LETIMER0);

	/* Enable LETIMER0 interrupt vector in NVIC*/
	NVIC_EnableIRQ(LETIMER0_IRQn);

	/* Start counting */
	LETIMER_Enable(LETIMER0, true);

}

void LETIMER0_end(void)
{
	LETIMER_IntDisable(LETIMER0, LETIMER_IF_UF);
	NVIC_DisableIRQ(LETIMER0_IRQn);
	unblockSleepMode(LETIMER_MIN_ENERGYSTATE);
}

/**************************************************************************//**
 * @brief LETIMER0_IRQHandler
 * Interrupt Service Routine for LETIMER
 *****************************************************************************/
void LETIMER0_IRQHandler(void)
{
	__disable_irq();
	//CORE_AtomicDisableIrq();

	/* IRQHandler Routine */
	uint32_t typeOfInterrupt = LETIMER_IntGet(LETIMER0);
	if((LETIMER_IF_COMP0 & typeOfInterrupt))
	{
		/* Clear LETIMER0 COMP0 interrupt flag */
		LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0);
		LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);

		g_external_event_status |= LETIMER0_EXT_EVT_COMP;

	}
		//If the sensor is on, then only call the getTemp function else do not do anything
//		if(I2C_TempSensor_isEnable())
//		{
//			int16_t tempValue = 0;
//			//enabling the IRQ as the I2C has an interrupt handler which is set to a higher priority than the LETIMER0
//			//which allows the I2C Int to be handled
//			__enable_irq();
//			int I2C_Status = I2C_TempSensor_GetTempSi7021(&tempValue);
//			__disable_irq();
//
//			if(I2C_Status == 0)	//i2cTransferDone has a value of 0
//			{
//				if(tempValue < g_temperatureThreshold)
//				{
//					GPIO_PinOutSet(LED1_port,LED1_pin);
//				}
//				/*else
//				{
//					GPIO_PinOutClear(LED1_port,LED1_pin);
//				}*/
//			}
//		}

	__enable_irq();
	//CORE_AtomicEnableIrq();

	//generating external event for the BLE
	gecko_external_signal(g_external_event_status);
}

/* The function calculates the required prescaled depending on the required period (msec).
 * It gets the clock frequency and calculates the prescaler
 * */
CMU_ClkDiv_TypeDef LETIMER0_calculateAndGetPrescaler(uint32_t period_msec)
{
	CMU_ClkDiv_TypeDef prescaler = 1;
	uint16_t counterMaxValue = 0xFFFF;
	uint32_t clockFreq = 0; /* In Hz */

	/*if(LETIMER_MIN_ENERGYSTATE < EM3)	// LFXO is selected Clock F - 32768 Hz
	{
		clockFreq = LFXO_CLOCK_FREQUENCY;
	}
	else	// ULFRCO is selected Clock F - 1000 Hz
	{
		clockFreq = ULFRCO_CLOCK_FREQUENCY;
	}
	*/
	clockFreq = CMU_ClockFreqGet(cmuClock_LFA);

	while( period_msec > (((counterMaxValue * prescaler *1000)/clockFreq)))
	{
		prescaler <<= 1;
	}
	return prescaler;

}
void LETIMER0_setCOMPValues(uint32_t period_msec, uint32_t onDutyCycle_msec)
{
	uint32_t countFor_Period = 0, countFor_OnDutyCycle = 0;
	uint32_t clockFreq = 0; /* In Hz */

	/*if(LETIMER_MIN_ENERGYSTATE < EM3)	// LFXO is selected Clock F - 32768 Mhz
	{
		clockFreq = LFXO_CLOCK_FREQUENCY;
	}
	else	// ULFRCO is selected Clock F - 1000 Hz
	{
		clockFreq = ULFRCO_CLOCK_FREQUENCY;
	}*/
	clockFreq = CMU_ClockFreqGet(cmuClock_LFA);
	CMU_ClkDiv_TypeDef prescalerValue = CMU_ClockDivGet(cmuClock_LETIMER0);

	countFor_Period = (period_msec * clockFreq) / (prescalerValue * 1000);
	countFor_OnDutyCycle = (onDutyCycle_msec * clockFreq) / (prescalerValue * 1000);

	LETIMER_CompareSet(LETIMER0, 0, countFor_Period);
	LETIMER_CompareSet(LETIMER0, 1, (countFor_Period - countFor_OnDutyCycle));
	g_currentOnDutyCycle_msec = onDutyCycle_msec;

}

void LETIMER0_increaseOnDutyCycle(uint32_t onDutyCycle_msec)
{
	int32_t effectiveOnDutyCycleValue = (int32_t)(g_currentOnDutyCycle_msec);
	effectiveOnDutyCycleValue += (int32_t)onDutyCycle_msec;
	if((effectiveOnDutyCycleValue < PERIOD_CYCLE_MILLISECONDS) && (effectiveOnDutyCycleValue > 0))
		LETIMER0_setCOMPValues(PERIOD_CYCLE_MILLISECONDS,(uint32_t)effectiveOnDutyCycleValue);
	else
		LETIMER0_setCOMPValues(PERIOD_CYCLE_MILLISECONDS,PERIOD_CYCLE_MILLISECONDS);
}

void LETIMER0_decreaseOnDutyCycle(uint32_t onDutyCycle_msec)
{
	int32_t effectiveOnDutyCycleValue = (int32_t)(g_currentOnDutyCycle_msec);
	effectiveOnDutyCycleValue -= (int32_t)onDutyCycle_msec;
	if(effectiveOnDutyCycleValue > 0)
		LETIMER0_setCOMPValues(PERIOD_CYCLE_MILLISECONDS,(uint32_t)effectiveOnDutyCycleValue);
	else
		LETIMER0_setCOMPValues(PERIOD_CYCLE_MILLISECONDS,0);
}

