/*
 * usart1.h
 *
 *  Created on: 28-Sep-2017
 *      Author: Gunj Manseta
 */

#ifndef SRC_SPI_H_
#define SRC_SPI_H_

#include "em_gpio.h"

#define BMA_EXT_EVT_DOUBLE_TAP		(0x02)
#define BMA_EXT_EVT_SINGLE_TAP		(0x04)
#define BMA_EXT_EVT_TAP				(0x100)
#define SPI_WAKEUPCOUNT_FOR_TIMER0 	(3600)

#define USART_0 (0)
#define USART_1 (1)

#define SPI_CS_PORT 	AF_USART1_CS_PORT(SPI_CS_LOCATION)		// maps to gpioPortC
#define SPI_CLK_PORT	AF_USART1_CLK_PORT(SPI_CLK_LOCATION)	// maps to gpioPortC
#define SPI_MISO_PORT	AF_USART1_RX_PORT(SPI_MISO_LOCATION)	// maps to gpioPortC
#define SPI_MOSI_PORT	AF_USART1_TX_PORT(SPI_MOSI_LOCATION)	// maps to gpioPortC


#define SPI_CS_PIN		AF_USART1_CS_PIN(SPI_CS_LOCATION) 		// maps to pin 9
#define SPI_CLK_PIN		AF_USART1_CLK_PIN(SPI_CLK_LOCATION) 	// maps to pin 8
#define SPI_MISO_PIN	AF_USART1_RX_PIN(SPI_MISO_LOCATION) 	// maps to pin 7
#define SPI_MOSI_PIN	AF_USART1_TX_PIN(SPI_MOSI_LOCATION) 	// maps to pin 6


/* Location found  from the BGM12x datasheet Page: 67,68 */
#define	SPI_MOSI_LOCATION 	_USART_ROUTELOC0_TXLOC_LOC11
#define	SPI_MISO_LOCATION 	_USART_ROUTELOC0_RXLOC_LOC11
#define	SPI_CS_LOCATION 	_USART_ROUTELOC0_CSLOC_LOC11
#define	SPI_CLK_LOCATION 	_USART_ROUTELOC0_CLKLOC_LOC11



// Register Values from the BMA280 datasheet that are stated in the questions.
#define FOUR_G_VALUE 				(0x05)
#define FOUR_G_VALUE_SHIFT 			(0)
#define FOUR_G_VALUE_SHIFTED 		(FOUR_G_VALUE << FOUR_G_VALUE_SHIFT)
#define G_VALUE_SELECT_REGISTER 	(0x0F)

#define BW_125_VALUE				(0x0C)
#define BW_VALUE_125_SHIFT 			(0)
#define BW_125_VALUE_SHIFTED 		(BW_125_VALUE << BW_VALUE_125_SHIFT)
#define BW_VALUE_SELECT_REGESTER	(0x10)

#define INT_OUT_CTRL_VALUE 			(0x01)
#define INT_OUT_CTRL_REGISTER 		(0x20)

#define POWER_NORMAL_MODE 			(0x00)
#define POWER_SUSPEND_MODE			(0x01 << 7)
#define POWER_DEEP_SUSPEND_MODE		(0x01 << 5)
#define POWER_MODE_SELECT_REGISTER	(0x11)


#define TAP_QUIET_DURATION_30ms		(0x00 << TAP_QUIET_SHIFT)
#define TAP_QUIET_DURATION_20ms		(0x01 << TAP_QUIET_SHIFT)
#define TAP_SHOCK_DURATION_50ms		(0x00 << TAP_SHOCK_SHIFT)
#define TAP_SHOCK_DURATION_75ms		(0x01 << TAP_SHOCK_SHIFT)
#define TAP_DURATION_250ms			(0x04 << TAP_DURATION_SHIFT)
#define TAP_DURATION_200ms			(0x03 << TAP_DURATION_SHIFT)
#define TAP_DURATION_SHIFT			(0)
#define TAP_SHOCK_SHIFT				(6)
#define TAP_QUIET_SHIFT				(7)
#define TIME_DEFINITION_REGISTER	(0x2A)

#define SAMPLE_VALUE_4							(0x01 << SAMPLE_VALUE_SHIFT)
#define TH_VALUE_250mg							(0x02 << TH_VALUE_SHIFT)
#define SAMPLE_VALUE_SHIFT						(6)
#define TH_VALUE_SHIFT							(0)
#define SAMPLE_AND_THRESHOLD_SELECT_REGISTER	(0x2B)

#define SINGLE_TAP_INT_EN			(0x01 << SINGLE_TAP_INT_SHIFT)
#define DOUBLE_TAP_INT_EN			(0x01 << DOUBLE_TAP_INT_SHIFT)
#define SINGLE_TAP_INT_DIS			(0x00 << SINGLE_TAP_INT_SHIFT)
#define DOUBLE_TAP_INT_DIS			(0x00 << DOUBLE_TAP_INT_SHIFT)
#define SINGLE_TAP_INT_SHIFT		(5)
#define DOUBLE_TAP_INT_SHIFT		(4)
#define TAP_INT_EN_REGISTER			(0x16)

#define TAP_SENSE_STATE_REGISTER	(0x09)
#define SINGLE_TAP_SENSED			(0x01 << SINGLE_TAP_SENSED_SHIFT)
#define DOUBLE_TAP_SENSED			(0x01 << DOUBLE_TAP_SENSED_SHIFT)
#define SINGLE_TAP_SENSED_SHIFT		(5)
#define DOUBLE_TAP_SENSED_SHIFT		(4)

#define SINGLE_TAP_MAP_INT1			(0x01 << SINGLE_TAP_MAP_INT1_SHIFT)
#define DOUBLE_TAP_MAP_INT1			(0x01 << DOUBLE_TAP_MAP_INT1_SHIFT)
#define SINGLE_TAP_MAP_INT1_SHIFT	(5)
#define DOUBLE_TAP_MAP_INT1_SHIFT	(4)
#define TAP_MAP_INT1_REGISTER		(0x19)

#define TAP_SOURCE_UNFILTERED		(0x01 << TAP_SOURCE_SHIFT)
#define TAP_SOURCE_SHIFT			(4)
#define TAP_SOURCE_REGISTER			(0x1E)

#define INT_LATCH_VALUE				(0x0C << INT_LATCH_VALUE_SHIFT)
#define INT_LATCH_VALUE_SHIFT		(0)
#define INT_RST_LATCH_REGISTER		(0x21)

//GPIO pin to which the EXT INT is routed
#define SPI_INT_PORT 				gpioPortD
#define SPI_INT_PIN					11



#define gpioModeMosi 	gpioModePushPull
#define gpioModeMiso 	gpioModeInput
#define gpioModeCs		gpioModePushPull
#define gpioModeClk  	gpioModePushPull

//SPI runs in the EM1 energy mode
#define ENERGY_MODE_SPI (EM1)

//***********************************************************************************
// function prototypes
//***********************************************************************************
void SPI_setup();
void SPI_start(bool enable);
void SPI_writeByteData(uint8_t registerAddress, uint8_t data);
uint8_t SPI_readByteData(uint8_t registerAddress);


#endif /* SRC_SPI_H_ */
