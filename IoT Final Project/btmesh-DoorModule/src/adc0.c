/*
 * adc0.c
 *
 *  Created on: 19-Sep-2017
 *      Author: Gunj Manseta
 */
//#include <sleep_g.h>
#include "adc0.h"
#include "em_adc.h"
#include "em_gpio.h"
//#include "gpio.h"
//#include "letimer0.h"
//#include "common.h"
//#include "spi.h"
#include "cmu.h"
//#include "timer0.h"
#include "native_gecko.h"
//***********************************************************************************
// defined macros
//***********************************************************************************
#define ADC_CMP_LT_VALUE        				3550                    /* ~2.8V for 3.3V AVDD */
#define ADC_CMP_GT_VALUE        				30                     	/* ~.02V for 3.3V AVDD */
#define JOYSTICK_INPUT							adcPosSelAPORT3XCH8    	/* PA0 */
#define ADC0_INPUT								JOYSTICK_INPUT
#define ADC_CMP_GT_FORDEBOUNCE					4000
#define ADC_CMP_LT_FORDEBOUNCE					4096
//#define ADC_CMP_FORDEBOUNCE_UPPERLIMIT			3800
//#define ADC_CMP_LT_FOR_JOYSTICK_CENTER_PRESS	50
//#define ADC_CMP_LT_FOR_JOYSTICK_NORTH_PRESS		3520
//#define ADC_CMP_LT_FOR_JOYSTICK_SOUTH_PRESS		2050
//#define ADC_CMP_LT_FOR_JOYSTICK_EAST_PRESS		3140
//#define ADC_CMP_LT_FOR_JOYSTICK_WEST_PRESS		2460
#define ADC_CMP_FORDEBOUNCE_UPPERLIMIT			4000
#define ADC_CMP_LT_FOR_JOYSTICK_CENTER_PRESS	90
#define ADC_CMP_LT_FOR_JOYSTICK_NORTH_PRESS		3900
#define ADC_CMP_LT_FOR_JOYSTICK_SOUTH_PRESS		2250
#define ADC_CMP_LT_FOR_JOYSTICK_EAST_PRESS		3340
#define ADC_CMP_LT_FOR_JOYSTICK_WEST_PRESS		2860
#define ADC_CLOCK_PRESCALE_VALUE				111

//***********************************************************************************
// global variables
//***********************************************************************************
volatile bool debounceFlag = true;
extern volatile uint32_t g_external_event_status;


//***********************************************************************************
// function defintions
//***********************************************************************************

//Only used internally by the ADC0_setup() function so does not have the declaration in the Header file
void ADC0_init(void)
{
	/* Setting up the clock tree for ADC0 */
	NVIC_DisableIRQ(ADC0_IRQn);
	ADC0->SINGLEFIFOCLEAR = ADC_SINGLEFIFOCLEAR_SINGLEFIFOCLEAR;
	cmu_init_ADC0();

	const ADC_Init_TypeDef init =
	{
		.ovsRateSel = 0,
		.timebase = ADC_TimebaseCalc(0),
		.em2ClockConfig = adcEm2ClockAlwaysOn,
		.prescale = ADC_CLOCK_PRESCALE_VALUE,
		.tailgate = 0,
		.warmUpMode = adcWarmupNormal

	};

	const ADC_InitSingle_TypeDef singleInit =
	{
		.posSel = ADC0_INPUT,
		.reference = adcRefVDD,
		.acqTime = adcAcqTime32,
		.rep = true,
		.fifoOverwrite = true,
		.prsEnable = false,
		.negSel = adcNegSelVSS,
		.resolution = adcRes12Bit,
		.diff = false,
		.prsSel = adcPRSSELCh0,
		.singleDmaEm2Wu = false,
		.leftAdjust = false
	};

	/* Change the below value if the ADC is misbehaving */
	ADC0->BIASPROG |= ADC_BIASPROG_GPBIASACC_LOWACC | ADC_BIASPROG_ADCBIASPROG_SCALE8;

	ADC_Init(ADC0, &init);
	ADC_InitSingle(ADC0, &singleInit);

}

void ADC0_setup(void)
{
	ADC0_init();

	/* Disabling the PA0 as it is needed for the Joystick input */
	GPIO_PinModeSet(gpioPortA,0,gpioModeDisabled,0);

	/* Enable single window compare */
	ADC0->SINGLECTRL |= ADC_SINGLECTRL_CMPEN;

	/* Initialize compare threshold for both single and scan conversion */
	ADC0->CMPTHR = _ADC_CMPTHR_RESETVALUE;
	ADC0->CMPTHR = (ADC_CMP_GT_VALUE << _ADC_CMPTHR_ADGT_SHIFT);
	ADC0->CMPTHR |= (ADC_CMP_LT_VALUE << _ADC_CMPTHR_ADLT_SHIFT);

	/* Set single data valid level (DVL) to trigger */
	//ADC0->SINGLECTRLX |= (ADC_SINGLE_DVL - 1) << _ADC_SINGLECTRLX_DVL_SHIFT;

}

void ADC0_start(void)
{
	//blockSleepMode(ENERGY_MODE_ADC0);

	/* Enable ADC Interrupt when window compare */
	ADC_IntEnable(ADC0, ADC_IEN_SINGLECMP);

	//Priority number is more than that of the TIMER0 i.e. TIMER0 INT has higher priority than ADC0 INT
	// Refer common.h for INT priority table
	//NVIC_SetPriority(ADC0_IRQn, IRQ_PRIORITY_ADC0);
	NVIC_ClearPendingIRQ(ADC0_IRQn);
	NVIC_EnableIRQ(ADC0_IRQn);

	/* Star the single ended Single mode ADC */
	ADC_Start(ADC0, adcStartSingle);
	//unblockSleepMode(ENERGY_MODE_ADC0);

}

void ADC0_stop()
{
	ADC_IntDisable(ADC0, ADC_IEN_SINGLECMP);
	NVIC_DisableIRQ(ADC0_IRQn);
}

void ADC0_IRQHandler(void)
{
	//blockSleepMode(ENERGY_MODE_ADC0);
	__disable_irq();
	//CORE_AtomicDisableIrq();

	uint32_t adcSampleValue = 0;

	if (ADC0->IF & ADC_IF_SINGLECMP)
	{
		ADC_IntClear(ADC0,ADC_IF_SINGLECMP);
		/* Read SINGLEDATA will clear SINGLE IF flag */
		adcSampleValue = ADC_DataSingleGet(ADC0);

		/* Debounce algorithm Used explained.
		 * Using a normal debouncing algorithm using a boolean flag and changing the Window Compare Values.
		 * Initial value of the debounce flag is set to false.
		 * On occurence of the very first button press, it executes the respective action associated with the button press.
		 * As there will be debouncing, the rest of the occurences of the button should be ignored unless the button is completely released.
		 * So, when the associated action gets completed, the debounce flag is set to true and
		 * the Window compare values is set to get the Interrupt for the Button release i.e. 3.3V ( adcSampleValue > 4000 )
		 * So, only the button release interrupt will come into the IRQ handler now.
		 * At this time, the debounce flag is true and the adcSample value > 4000.
		 * Here, we reset the debounce flag again, and change the window compare value back to normal to detect the joystick button press.
		 *
		 */
		if(!debounceFlag)
		{
			//center press
			if(adcSampleValue < ADC_CMP_LT_FOR_JOYSTICK_CENTER_PRESS)
			{
				g_external_event_status |= ADC_EXT_EVT_CENTER;
				gecko_external_signal(g_external_event_status);
			}
			//south press
			else if(adcSampleValue < ADC_CMP_LT_FOR_JOYSTICK_SOUTH_PRESS)
			{
				g_external_event_status |= ADC_EXT_EVT_SOUTH;
				gecko_external_signal(g_external_event_status);
			}
			//west press
			else if(adcSampleValue < ADC_CMP_LT_FOR_JOYSTICK_WEST_PRESS)
			{
				g_external_event_status |= ADC_EXT_EVT_WEST;
				gecko_external_signal(g_external_event_status);
			}
			//east press
			else if(adcSampleValue < ADC_CMP_LT_FOR_JOYSTICK_EAST_PRESS)
			{
				g_external_event_status |= ADC_EXT_EVT_EAST;
				gecko_external_signal(g_external_event_status);
			}
			//north press
			else if(adcSampleValue < ADC_CMP_LT_FOR_JOYSTICK_NORTH_PRESS)
			{
				g_external_event_status |= ADC_EXT_EVT_NORTH;

				gecko_external_signal(g_external_event_status);
			}
		}
		else if(adcSampleValue > ADC_CMP_FORDEBOUNCE_UPPERLIMIT)
		{
			//changing back the window compare values to detect the joystick button press
			ADC_window_in();
			debounceFlag = false;
		}
	}

	ADC0->SINGLEFIFOCLEAR = ADC_SINGLEFIFOCLEAR_SINGLEFIFOCLEAR;
	__enable_irq();
	//CORE_AtomicEnableIrq();
}

/*
 * Generates automatic prescaler for the given sample, Acq Time, Resolution and Oversampleing.
 * To be done.
*/
uint32_t ADC0_calculateAndGetPrescaler(uint32_t samplesPerSecond)
{
	uint32_t adc0Prescaler = 1;

	//uint32_t ADCClockFreq = CMU_ClockFreqGet(cmuClock_ADC0);

	return adc0Prescaler;
}

void ADC_window_out()
{
//	ADC0->CMPTHR = (ADC_CMP_LT_VALUE << _ADC_CMPTHR_ADGT_SHIFT);
//	ADC0->CMPTHR |= (ADC_CMP_GT_VALUE << _ADC_CMPTHR_ADLT_SHIFT);

	ADC0->CMPTHR = (ADC_CMP_GT_FORDEBOUNCE << _ADC_CMPTHR_ADGT_SHIFT);
	ADC0->CMPTHR |= (ADC_CMP_LT_FORDEBOUNCE << _ADC_CMPTHR_ADLT_SHIFT);
}

void ADC_window_in()
{
	ADC0->CMPTHR = (ADC_CMP_GT_VALUE << _ADC_CMPTHR_ADGT_SHIFT);
	ADC0->CMPTHR |= (ADC_CMP_LT_VALUE << _ADC_CMPTHR_ADLT_SHIFT);
}
