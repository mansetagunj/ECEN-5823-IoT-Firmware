/*
 * i2c.h
 *
 *  Created on: 02-Oct-2017
 *      Author: Gunj Manseta
 */

#ifndef SRC_I2C_TEMPSENSOR_H_
#define SRC_I2C_TEMPSENSOR_H_

#include "gpio.h"

#define TEMPSENSOR_WAKEUPCOUNT_FOR_TIMER0 (36000)

#define I2C0_MIN_ENERGYSTATE (EM1)

#define TEMP_SENSOR_ADDR 		(0x40)
#define IO_EXPANDER_ADDR		(0x90)

#define SENSOR_CTRL_REG_ADDR	(0x02)
#define SENSOR_ENABLE			(0x01)

#define TEMP_SENSOR_MEASURE_TEMP_CMD	(0xE3)
#define TEMP_SENSOR_RESET_CMD			(0xFE)
#define TEMP_SENSOR_WRITE_USR_REG1_CMD	(0xE6)
#define TEMP_SENSOR_READ_USR_REG1_CMD	(0xE7)

#define TEMP_SENSOR_WRITE_USR_REG1_DATA	(0x00) //14bit res, On chip heater - disable

#define I2C0_SCL_PIN 	AF_I2C0_SCL_PIN(_I2C_ROUTELOC0_SCLLOC_LOC14)
#define I2C0_SDA_PIN 	AF_I2C0_SDA_PIN(_I2C_ROUTELOC0_SDALOC_LOC16)
#define I2C0_SCL_PORT	AF_I2C0_SCL_PORT(_I2C_ROUTELOC0_SCLLOC_LOC14)
#define I2C0_SDA_PORT 	AF_I2C0_SDA_PORT(_I2C_ROUTELOC0_SDALOC_LOC16)

void I2C_TempSensor_setup();
void I2C_TempSensor_Enable(bool enable);
unsigned int I2C_TempSensor_isEnable();
int I2C_TempSensor_GetTempSi7021(float /*int16_t*/ *inOutTempValue);
int I2C_TempSensor_WakeUp();
void I2C_TempSensor_Sleep();


#endif /* SRC_I2C_TEMPSENSOR_H_ */
