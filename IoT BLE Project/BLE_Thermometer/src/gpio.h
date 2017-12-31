//***********************************************************************************
// Include files
//***********************************************************************************
#include "em_gpio.h"
//***********************************************************************************
// defined files
//***********************************************************************************

#define gpioPin4 4
#define gpioPin5 5

// LED0 pin
#define	LED0_port gpioPortF //5 //Port F is mapped as number 5
#define LED0_pin gpioPin4 //4 //LED0 is connected to PF4 i.e. Port F pin 4
#define LED0_default	false 	// off
// LED1 pin
#define LED1_port gpioPortF //5 //Port F is mapped as number 5
#define LED1_pin gpioPin5 //5 //LED1 is connected to PF5 i.e. Port F pin 5
#define LED1_default	false	// off


#define GPIO_SENSORENABLE_PIN	(9)
#define GPIO_SENSORENABLE_PORT	(gpioPortD)

//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void gpio_LETIMER0_init(void);
void gpio_SPI_init();
