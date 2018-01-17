//***********************************************************************************
// Include files
//***********************************************************************************
#include "cmu.h"
#include "common.h"

//#include "letimer0.h"
//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************

//***********************************************************************************
// function defintions
//***********************************************************************************
void cmu_init(void)
{
	 //By default, HFRCO is enabled  cmuHFRCOFreq_19M0Hz
//	CMU_HFRCOBandSet(cmuHFRCOFreq_2M0Hz); 				// Set HFRCO frequency
//
//	CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);		// Enabling the HFRCO as the enter_DefaultMode_from_RESET() function diables it somehow
//	CMU_HFXOAutostartEnable(0, false, false);
//	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
//
//	CMU_OscillatorEnable(cmuOsc_HFXO, false, false);	// Disable HFXO
//
//	 //By default, LFRCO is enabled
//	CMU_OscillatorEnable(cmuOsc_LFRCO, false, false);

	//CMU_ClockSelectSet(cmuClock_HFPER,cmuSelect_HFXO);
	//CMU_OscillatorEnable(cmuOsc_LFXO, false, false);		// Disable LFXO

	//uint32_t clk = CMU_ClockFreqGet(cmuClock_HFPER);
	//clk++;
	CMU_ClockEnable(cmuClock_HFPER, true);
	// Peripheral clocks enabled
	CMU_ClockEnable(cmuClock_GPIO, true);

	CMU_ClockEnable(cmuClock_HFLE, true);                           //enabling the coreLE clock

}

/*
 *CMU clock init specifically for LETIMER0
 */
void cmu_init_LETIMER0(CMU_ClkDiv_TypeDef prescaler)
{
	/* For LETIMER in ENERGY STATE EM0-EM2, LFXO should be used.
	 * For EM3, ULFRCO should be used.
	 */
	if(LETIMER_MIN_ENERGYSTATE < EM3)
	{
		CMU_OscillatorEnable(cmuOsc_LFXO, true, true);			// Enable LFXO
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);		// Select LFXO as the source of LFACLK
		//prescaler = cmuClkDiv_2;								// Hard coding the value due to less time in hand. Modify it to make it more dynamic
	}
	else
	{
		CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);		// Enable ULFRCO
		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);		// Select ULFRCO as the source of LFACLK
		//prescaler = cmuClkDiv_1;								// Hard coding the value due to less time in hand. Modify it to make it more dynamic
	}

	CMU_OscillatorEnable(cmuOsc_LFRCO,false,false);				// Disable LFRCO
	CMU_ClockEnable(cmuClock_CORELE, true);

	// Peripheral clocks enabled
	CMU_ClockEnable(cmuClock_LETIMER0, true);

	/* Setting up Prescaler for LETIMER0 clock */
	CMU_ClockDivSet(cmuClock_LETIMER0, prescaler);
}

void cmu_init_ADC0()
{
	/* Setting up the AUXHFRCO clock */
	CMU_AUXHFRCOBandSet(CMU_AUXHFRCO_MIN);
	CMU_OscillatorEnable(cmuOsc_AUXHFRCO, true, true);
	CMU_ClockSelectSet(cmuClock_AUX,cmuSelect_AUXHFRCO);
	//ADC0->CTRL |= ADC_CTRL_ADCCLKMODE_ASYNC | ADC_CTRL_ASYNCCLKEN_ASNEEDED;

	//Setting up the ADCCTRL register and setting up the ADC clock to AUXHFRCO
	CMU->ADCCTRL  = CMU_ADCCTRL_ADC0CLKSEL_AUXHFRCO;

	CMU_ClockSelectSet(cmuClock_ADC0,cmuSelect_AUXHFRCO);

	/* Enable ADC clock */
	CMU_ClockEnable(cmuClock_ADC0, true);
}

void cmu_init_TIMER0()
{
	//CMU_ClockSelectSet(cmuClock_HFPER,cmuSelect_HFRCO);

	CMU_ClockEnable(cmuClock_HFPER,true);

	/* Enable clock for TIMER0 */
	CMU_ClockEnable(cmuClock_TIMER0, true);

}

void cmu_init_USARTn(uint8_t USART_NUM)
{
	//CMU_ClockSelectSet(cmuClock_HFPER,cmuSelect_HFRCO);

	switch (USART_NUM) {
		case 0:
			/* Enable the clock for USART0 */
			CMU_ClockEnable(cmuClock_USART0,true);
			break;
		case 1:
		default:
			/* Enable the clock for USART1 */
			CMU_ClockEnable(cmuClock_USART1,true);
			break;
	}

	CMU_ClockEnable(cmuClock_GPIO, true);
}
void cmu_init_I2C0()
{
	//CMU_ClockSelectSet(cmuClock_HFPER,cmuSelect_HFRCO);
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_I2C0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
}
