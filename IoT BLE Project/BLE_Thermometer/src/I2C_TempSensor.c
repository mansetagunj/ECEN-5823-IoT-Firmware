/*
 * i2c.c
 *
 *  Created on: 02-Oct-2017
 *      Author: Gunj Manseta
 */
#include "I2C_TempSensor.h"
#include "cmu.h"
#include "em_i2c.h"
#include "em_gpio.h"
#include "sleep_g.h"
#include "common.h"
#include "timer0.h"
#include "gpio.h"
#include "i2cspm.h"

//volatile int16_t g_temperatureThreshold = 45;
volatile float  g_temperatureThreshold = 15;

volatile I2C_TransferReturn_TypeDef g_I2C_Status;


void I2C_TempSensor_init()
{
	I2C_Init_TypeDef i2cInit =
	{
		  .enable 	= false,                 // Do not Enable when init done
		  .master 	= true,                  // Set to master mode
		  .refFreq 	= 0,                     // Use currently configured reference clock
		  .freq 	= I2C_FREQ_STANDARD_MAX, // Set to standard rate assuring being within I2C spec
		  .clhr 	= i2cClockHLRStandard    // Set to use 4:4 low/high duty cycle
	};

	I2C_Init(I2C0, &i2cInit);
}

//Writes to I2C slave register
int I2C_TempSensor_RegWrite(I2C_TypeDef *i2c,
                     	 	 uint8_t slaveAddr,
							 uint8_t registerWrite_cmd,
							 uint8_t registerWrite_data)
{
	I2C_TransferSeq_TypeDef seq;
	//using this temp local data variable as the seq structure has a pointer to hold the data pointer
	// and we don't want the incoming register_cmd or register_data to go out of scope and cause issue.
	uint8_t data[2] = {0};

	/*if (reg == tempsensRegTemp)
	{
	return(-1);
	}*/

	//making sure the R/W bit (LSB bit) is zero
	//the slave address should be in D7-D1 bits of the slaveAddr variable
	seq.addr = (uint16_t)((slaveAddr<<1) & ((uint8_t)0xFE));

	/* Select register to be written */
	data[0] = registerWrite_cmd;
	seq.buf[0].data = data;
	seq.buf[0].len = 1;

	if(TEMP_SENSOR_WRITE_USR_REG1_CMD == registerWrite_cmd)
	{
		//S+Seq(ADDR(W))+Seq(BUF0(DATA))+Seq(BUF1(DATA))+P -> Start+SlaveAddr(W)+registerWrite_cmd+registerWrite_data+Stop
		seq.flags = I2C_FLAG_WRITE_WRITE;

		//The data that needs to be written
		data[1] = registerWrite_data;
		seq.buf[1].data = data+1;
		seq.buf[1].len = 1;
	}
	else if(TEMP_SENSOR_RESET_CMD == registerWrite_cmd)
	{
		//S+Seq(ADDR(W))+Seq(BUF0(DATA)+P -> Start+SlaveAddr(W)+ cmdValue + Stop
		seq.flags = I2C_FLAG_WRITE;

		//No data needs to be written do setting the buf1 struct data to zero
		data[1] = 0;
		seq.buf[1].data = data+1;
		seq.buf[1].len = 1;
	}
	else if(SENSOR_CTRL_REG_ADDR == registerWrite_cmd && IO_EXPANDER_ADDR == slaveAddr)
	{
		//S+Seq(ADDR(W))+Seq(BUF0(DATA))+Seq(BUF1(DATA))+P -> Start+SlaveAddr(W)+registerWrite_cmd+registerWrite_data+Stop
		seq.flags = I2C_FLAG_WRITE_WRITE;

		//The data that needs to be written
		data[1] = registerWrite_data;
		seq.buf[1].data = data+1;
		seq.buf[1].len = 1;
	}
	else
	{
		//Not providing support for any other commands other than User register write and Reset
		return (-1);
	}

	g_I2C_Status = I2C_TransferInit(i2c, &seq);

	while (g_I2C_Status == i2cTransferInProgress);

	//I2C_Transfer function will be called in the IRQ handler

	return(g_I2C_Status);
}

int I2C_TempSensor_RegRead(I2C_TypeDef *i2c,
                         uint8_t slaveAddr,
                         uint8_t tempGet_cmd,
                         uint16_t *outTempData)
{
	// No support for anything else than getting the temp sensor value
	if(TEMP_SENSOR_MEASURE_TEMP_CMD != tempGet_cmd)
		return (-1);

	I2C_TransferSeq_TypeDef seq;
	//using this temp local data variable as the seq structure has a pointer to hold the data pointer
	// and we don't want the incoming register_cmd or register_data to go out of scope and cause issue.
	uint8_t data[3] = {0};


	//making sure the R/W bit (LSB bit) is zero
	//the slave address should be in D7-D1 bits of the slaveAddr variable
	seq.addr = (uint16_t)((slaveAddr<<1) & ((uint8_t)0xFE));

	data[0] = tempGet_cmd;
	seq.buf[0].data = data;
	seq.buf[0].len = 1;

	seq.flags = I2C_FLAG_WRITE_READ;

	//The received data buffer should be of 2 bytes as the temp data received is of 2 bytes
    seq.buf[1].data = data+1;
    seq.buf[1].len = 2;

    //g_I2C_Status = I2C_TransferInit(i2c, &seq);
    //g_I2C_Status = I2CSPM_Transfer(i2c,&seq);
    uint32_t timeout = I2CSPM_TRANSFER_TIMEOUT;
    g_I2C_Status = I2C_TransferInit(i2c, &seq);
    while (g_I2C_Status == i2cTransferInProgress && timeout--)
    {
    	g_I2C_Status = I2C_Transfer(i2c);
    }
    //return g_I2C_Status;
    //while (g_I2C_Status == i2cTransferInProgress);

	if (g_I2C_Status != i2cTransferDone)
	{
		return g_I2C_Status;
	}

	*outTempData = (uint16_t)(((uint16_t)data[1]) << 8);
	//uint16_t tempVal = *outTempData;
	*outTempData = ((*outTempData) | /*(uint16_t)*/(data[2] & 0xFC));
	//tempVal = *outTempData;
	return g_I2C_Status;
}

int  I2C_TempSensor_GetTempSi7021(float /*int16_t*/ *inOutTempValue)
{
	int ret = 0;

	if(I2C_TempSensor_isEnable())
	{
		blockSleepMode(I2C0_MIN_ENERGYSTATE);
		uint16_t tempSensorData = 0;
		g_I2C_Status = I2C_TempSensor_RegRead(I2C0,TEMP_SENSOR_ADDR,TEMP_SENSOR_MEASURE_TEMP_CMD,&tempSensorData);

		if (g_I2C_Status < 0)
		{
			return g_I2C_Status;
		}

		//uint32_t temperatureCode = 0;
		float temperatureCode = 0;
		temperatureCode = /*(uint32_t)*/(tempSensorData);

		//temperatureCode = ((((17572 * temperatureCode) >> 16) - 4685)>>2)/25;
		temperatureCode = (((17572 * temperatureCode) / 65536) - 4685)/100;
		*inOutTempValue = /*(int16_t)*/temperatureCode;
		unblockSleepMode(I2C0_MIN_ENERGYSTATE);

		ret = g_I2C_Status;
	}

	return ret;
}
void I2C_TempSensor_Configure()
{
	//enabling the Sensor Module on
	//g_I2C_Status = I2C_TempSensor_RegWrite(I2C0,IO_EXPANDER_ADDR,SENSOR_CTRL_REG_ADDR,SENSOR_ENABLE);

	g_I2C_Status = I2C_TempSensor_RegWrite(I2C0,TEMP_SENSOR_ADDR,TEMP_SENSOR_WRITE_USR_REG1_CMD,TEMP_SENSOR_WRITE_USR_REG1_DATA);

}

void I2C0_BusReset()
{
	//Used to reset the I2C bus
	for (int i = 0; i < 9; i++)
	{
		GPIO_PinOutClear(I2C0_SCL_PORT, I2C0_SCL_PIN);
		GPIO_PinOutSet(I2C0_SCL_PORT, I2C0_SCL_PIN);
	}
}

int I2C_TempSensor_WakeUp()
{
	I2C_TempSensor_Enable(true);

	//using timer0 for giving 18ms wakeup time for the Temp Sensor to get ready
	//TIMER0_startwithCount(TEMPSENSOR_WAKEUPCOUNT_FOR_TIMER0);
	TIMER0_startwithCount(47904);
	while(!TIMER_INT_SERVED);
	TIMER_INT_SERVED = false;

	//Use location 14 for SCL - PC10 and location 16 for SDA - PC11
	//Using gpioWiredAnd mode to avoid damage to other peripherals
	GPIO_PinModeSet(I2C0_SCL_PORT, I2C0_SCL_PIN, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(I2C0_SDA_PORT, I2C0_SDA_PIN, gpioModeWiredAnd, 1);

	I2C0_BusReset();

	I2C_Enable(I2C0,true);


	if(I2C0->STATE & I2C_STATE_BUSY)
		I2C0->CMD = I2C_CMD_ABORT;


	return I2C_TempSensor_isEnable();

}

void I2C_TempSensor_Sleep()
{
	GPIO_PinModeSet(I2C0_SCL_PORT, I2C0_SCL_PIN, gpioModeDisabled, 1);
	GPIO_PinModeSet(I2C0_SDA_PORT, I2C0_SDA_PIN, gpioModeDisabled, 1);

	I2C_TempSensor_Enable(false);
}

void I2C_TempSensor_setup()
{
	cmu_init_I2C0();

	//Sensor enable pin mode.
	GPIO_PinModeSet(GPIO_SENSORENABLE_PORT, GPIO_SENSORENABLE_PIN, gpioModePushPull, 0);

	/* Enable pins at location 14 and 16*/
	I2C0->ROUTELOC0 = I2C_ROUTELOC0_SCLLOC_LOC14 | I2C_ROUTELOC0_SDALOC_LOC16;
	I2C0->ROUTEPEN = (I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN);

	I2C_TempSensor_init();

	/* Clear and enable interrupt from I2C module */
	NVIC_ClearPendingIRQ(I2C0_IRQn);

	//Setting the INT priority higher than the LETIMER0 (which has priority number of 4) as I2C is using interrupt
	// to send/receive the data to/from the Si7021. Refer common.h for peripheral priorities that are set
	//NVIC_SetPriority(I2C0_IRQn, IRQ_PRIORITY_I2C0);

	//NVIC_EnableIRQ(I2C0_IRQn);

}

unsigned int I2C_TempSensor_isEnable()
{
	return GPIO_PinOutGet(GPIO_SENSORENABLE_PORT, GPIO_SENSORENABLE_PIN);
}

void I2C_TempSensor_Enable(bool enable)
{
	if(enable)
	{
		GPIO_PinOutSet(GPIO_SENSORENABLE_PORT, GPIO_SENSORENABLE_PIN);
	}
	else
	{
		GPIO_PinOutClear(GPIO_SENSORENABLE_PORT, GPIO_SENSORENABLE_PIN);
	}
}

void I2C0_IRQHandler()
{
		__disable_irq();
		//Just run the I2C_Transfer function that checks interrupts flags and returns the appropriate status
		g_I2C_Status = I2C_Transfer(I2C0);
		__enable_irq();
}

/**********************************************/


