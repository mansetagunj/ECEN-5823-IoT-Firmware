/*
 * usart1.c
 *
 *  Created on: 28-Sep-2017
 *      Author: Gunj Manseta
 */
#include "sleep_g.h"
#include "spi.h"
#include "em_usart.h"
#include "cmu.h"
#include "gpio.h"
#include "timer0.h"
#include "I2C_TempSensor.h"
#include "common.h"
#include "native_gecko.h"

void SPI_init()
{
	/* Initialize with default settings and then update fields according to application requirements. */
	const USART_InitSync_TypeDef initSync =
	{
	    .enable = usartDisable,			/* Do not enable RX/TX when init completed. */
	    .refFreq = 0,               	/* Use current configured reference clock for configuring baudrate. */
	    .baudrate = 1000000,
	    .databits = usartDatabits8,  	/* 8 databits. */
	    .master = true,            		/* Master mode. */
	    .msbf = true,           		/* Send least significant bit first. */
	    .clockMode = usartClockMode3,
	    .prsRxEnable = false,           /* Not USART PRS input mode. */
	    .prsRxCh = usartPrsRxCh0,   	/* PRS channel 0. */
	    .autoTx = false,           		/* No AUTOTX mode. */
	    .autoCsEnable = true,          	/* AUTOCS mode */
	    .autoCsHold = 1,               	/* Auto CS Hold cycles */
	    .autoCsSetup = 1                /* Auto CS Setup cycles */
	};

	USART_InitSync(USART1, &initSync);
}

void SPI_setup()
{
	cmu_init_USARTn(USART_1);

	gpio_SPI_init();

	/* Set GPIO configuration to master */
	/*GPIO_Mode_TypeDef gpioModeMosi = gpioModePushPull;
	GPIO_Mode_TypeDef gpioModeMiso = gpioModeInput;
	GPIO_Mode_TypeDef gpioModeCs   = gpioModePushPull;
	GPIO_Mode_TypeDef gpioModeClk  = gpioModePushPull;*/

	GPIO_PinModeSet(SPI_MOSI_PORT, SPI_MOSI_PIN, gpioModeMosi, 1); /* MOSI */
	GPIO_PinModeSet(SPI_MISO_PORT, SPI_MISO_PIN, gpioModeMiso, 0); /* MISO */
	GPIO_PinModeSet(SPI_CS_PORT, SPI_CS_PIN, gpioModeCs,   1); /* CS */
	GPIO_PinModeSet(SPI_CLK_PORT, SPI_CLK_PIN, gpioModeClk,  1); /* Clock */

	SPI_init();

	/* Routing the LOC0 to appropriate Pins */
	USART1->ROUTELOC0 = (USART1->ROUTELOC0 &
							~(	_USART_ROUTELOC0_TXLOC_MASK | _USART_ROUTELOC0_RXLOC_MASK |
								_USART_ROUTELOC0_CLKLOC_MASK| _USART_ROUTELOC0_CSLOC_MASK
							)
						 )
						 | (SPI_MOSI_LOCATION << _USART_ROUTELOC0_TXLOC_SHIFT)
						 | (SPI_MISO_LOCATION << _USART_ROUTELOC0_RXLOC_SHIFT)
						 | (SPI_CLK_LOCATION<< _USART_ROUTELOC0_CLKLOC_SHIFT)
						 | (SPI_CS_LOCATION<<_USART_ROUTELOC0_CSLOC_SHIFT);


	/*Routing enable TX, RX, CLK, and CS pin for USART1 use */
	USART1->ROUTEPEN = (USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_CSPEN | USART_ROUTEPEN_CLKPEN );

	USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;

}

/*Configuring all the required register values in the BMA280 by writing data onto the SPI bus */
void BMA280_configure()
{
	SPI_writeByteData(G_VALUE_SELECT_REGISTER,FOUR_G_VALUE);
	SPI_readByteData(G_VALUE_SELECT_REGISTER);
	SPI_writeByteData(BW_VALUE_SELECT_REGESTER, BW_125_VALUE_SHIFTED);
	SPI_readByteData(BW_VALUE_SELECT_REGESTER);
	SPI_writeByteData(TIME_DEFINITION_REGISTER, TAP_QUIET_DURATION_30ms|TAP_SHOCK_DURATION_50ms|TAP_DURATION_200ms);
	SPI_readByteData(TIME_DEFINITION_REGISTER);
	SPI_writeByteData(SAMPLE_AND_THRESHOLD_SELECT_REGISTER, SAMPLE_VALUE_4| TH_VALUE_250mg);
	SPI_readByteData(SAMPLE_AND_THRESHOLD_SELECT_REGISTER);

	//SPI_writeByteData(TAP_INT_EN_REGISTER, SINGLE_TAP_INT_DIS | DOUBLE_TAP_INT_EN );		//enabling double tap Interrupt
	SPI_writeByteData(TAP_INT_EN_REGISTER, SINGLE_TAP_INT_EN | DOUBLE_TAP_INT_EN );		//enabling single tap Interrupt
	SPI_readByteData(TAP_INT_EN_REGISTER);

	GPIO_PinModeSet(SPI_INT_PORT, SPI_INT_PIN, gpioModeInput, 1);		//gpio port D pin 11 for interrupt

	//SPI_writeByteData(TAP_MAP_INT1_REGISTER, DOUBLE_TAP_MAP_INT1); 	//mapping INT1 to double TAP
	SPI_writeByteData(TAP_MAP_INT1_REGISTER, SINGLE_TAP_MAP_INT1 | DOUBLE_TAP_MAP_INT1); 	//mapping INT1 to single TAP
	SPI_readByteData(TAP_MAP_INT1_REGISTER);
//	SPI_writeByteData(TAP_SOURCE_REGISTER, TAP_SOURCE_UNFILTERED);
//	SPI_readByteData(TAP_SOURCE_REGISTER);
//	SPI_writeByteData(INT_RST_LATCH_REGISTER, INT_LATCH_VALUE);
//	SPI_readByteData(INT_RST_LATCH_REGISTER);
//	SPI_writeByteData(INT_OUT_CTRL_REGISTER, INT_OUT_CTRL_VALUE);
//	SPI_readByteData(INT_OUT_CTRL_REGISTER);

}

void SPI_start(bool normalMode)
{
	blockSleepMode(ENERGY_MODE_SPI);

	USART_Enable(USART1, usartEnable);

	if(normalMode)
	{
		NVIC_DisableIRQ(GPIO_ODD_IRQn);

		GPIO_PinModeSet(SPI_MOSI_PORT, SPI_MOSI_PIN, gpioModeMosi, 1); /* MOSI */
		GPIO_PinModeSet(SPI_MISO_PORT, SPI_MISO_PIN, gpioModeMiso, 0); /* MISO */
		GPIO_PinModeSet(SPI_CS_PORT, SPI_CS_PIN, gpioModeCs,   1); /* CS */
		GPIO_PinModeSet(SPI_CLK_PORT, SPI_CLK_PIN, gpioModeClk,  1); /* Clock */

		SPI_writeByteData(POWER_MODE_SELECT_REGISTER,POWER_NORMAL_MODE);

		//starting the timer for generating the delay of 1.8ms
		//TIMER0_startwithCount(SPI_WAKEUPCOUNT_FOR_TIMER0);
		TIMER0_startwithCount(1900);
		while(!TIMER_INT_SERVED);
		TIMER_INT_SERVED = false;


		BMA280_configure();
		//GPIO_IntDisable(SPI_INT_PORT);
		GPIO_IntConfig(SPI_INT_PORT, SPI_INT_PIN,true,false, true);
		//GPIO_IntClear(SPI_INT_PORT);
		GPIO_IntEnable(SPI_INT_PORT);
		NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
		NVIC_SetPriority(GPIO_ODD_IRQn,5);
		NVIC_EnableIRQ(GPIO_ODD_IRQn);
	}
	else
	{
		//SPI_writeByteData(POWER_MODE_SELECT_REGISTER,POWER_SUSPEND_MODE);
		SPI_writeByteData(POWER_MODE_SELECT_REGISTER,POWER_DEEP_SUSPEND_MODE);

		GPIO_PinModeSet(SPI_MOSI_PORT, SPI_MOSI_PIN, gpioModeDisabled, 1); /* MOSI */
		GPIO_PinModeSet(SPI_MISO_PORT, SPI_MISO_PIN, gpioModeDisabled, 0); /* MISO */
		GPIO_PinModeSet(SPI_CS_PORT, SPI_CS_PIN, gpioModeDisabled,   1); /* CS */
		GPIO_PinModeSet(SPI_CLK_PORT, SPI_CLK_PIN, gpioModeDisabled,  1); /* Clock */

		GPIO_IntClear(SPI_INT_PORT);
		GPIO_IntDisable(SPI_INT_PORT);
		NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
		NVIC_DisableIRQ(GPIO_ODD_IRQn);
		USART_Enable(USART1, usartDisable);
	}

	unblockSleepMode(ENERGY_MODE_SPI);
}


uint8_t SPI_readByteData(uint8_t registerAddress)
{
	__disable_irq();
	USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;

	uint32_t dataRead;

	/* Waiting for the USART1 to be ready */
	while (!(USART_StatusGet(USART1) & USART_STATUS_TXBL));

	//Sending Dummy Data to generate the clock and send the register address to read data from
	USART1->TXDOUBLE = (uint32_t)(registerAddress | (0x80));

	/* Wait for TX to complete */
	while(!(USART_StatusGet(USART1)& USART_STATUS_TXC));

	// Wait for valid RX DATA
	while(!(USART_StatusGet(USART1)& USART_STATUS_RXDATAV));

	dataRead = USART1->RXDOUBLE;
	dataRead = (dataRead >> 8);

	__enable_irq();
	return dataRead;
}


 void SPI_writeByteData(uint8_t registerAddress, uint8_t data)
{
	__disable_irq();
	USART1->CMD |= USART_CMD_CLEARTX | USART_CMD_CLEARRX;

	uint32_t dataToSend;

	dataToSend = (uint32_t)(registerAddress | (data << 8));
	/* Waiting for the USART1 to be ready */
	while (!(USART_StatusGet(USART1) & USART_STATUS_TXBL));

	USART1->TXDOUBLE =	dataToSend;

	/* Wait for TX to complete */
	while(!(USART_StatusGet(USART1) & USART_STATUS_TXC));

	(USART1->RXDOUBLE);
	__enable_irq();
}

void GPIO_ODD_IRQHandler()
{
	__disable_irq();
	blockSleepMode(ENERGY_MODE_SPI);

	GPIO_IntClear(0xFFFF);

	g_external_event_status |= BMA_EXT_EVT_TAP;

	unblockSleepMode(ENERGY_MODE_SPI);
	__enable_irq();
	gecko_external_signal(g_external_event_status);
}




