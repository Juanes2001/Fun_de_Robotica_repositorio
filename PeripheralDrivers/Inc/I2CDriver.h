/*
 * I2CDriver.c
 *
 *  Created on: 23 oct. 2022
 *      Author: JUAN ESTEBAN
 */


#ifndef I2CDRIVER_H_
#define I2CDRIVER_H_

#include <stm32f4xx.h>

#define I2C_WRITE_DATA    0
#define I2C_READ_DATA     1

#define MAIN_CLOCK_4_MHz_FOR_I2C   4
#define MAIN_CLOCK_16_MHz_FOR_I2C  16
#define MAIN_CLOCK_20_MHz_FOR_I2C  20
#define MAIN_CLOCK_50_MHz_FOR_I2C  50


#define I2C_MODE_SM    0
#define I2C_MODE_FM    1


#define I2C_MODE_SM_SPEED_100KHz_4MHz  	20
#define I2C_MODE_SM_SPEED_100KHz_16MHz  80
#define I2C_MODE_SM_SPEED_100KHz_20MHz  100
#define I2C_MODE_SM_SPEED_100KHz_50MHz  250


#define I2C_MODE_FM_SPEED_400KHz_4MHz  	3
#define I2C_MODE_FM_SPEED_400KHz_16MHz  13
#define I2C_MODE_FM_SPEED_400KHz_20MHz  17
#define I2C_MODE_FM_SPEED_400KHz_50MHz  42


#define I2C_MAX_RISE_TIME_SM_4MHz       5
#define I2C_MAX_RISE_TIME_SM_16MHz      17
#define I2C_MAX_RISE_TIME_SM_20MHz      21
#define I2C_MAX_RISE_TIME_SM_50MHz      51


#define I2C_MAX_RISE_TIME_FM_4MHz       2
#define I2C_MAX_RISE_TIME_FM_16MHz      6
#define I2C_MAX_RISE_TIME_FM_20MHz      7
#define I2C_MAX_RISE_TIME_FM_50MHz      16


typedef struct
{
	uint8_t       slaveAddress;
	uint8_t		  modeI2C;
	uint8_t       dataI2C;
	uint8_t       clkSpeed;

}I2C_Config_t;

typedef struct
{
	I2C_TypeDef   *ptrI2Cx;
	I2C_Config_t  I2C_Config;
}I2C_Handler_t;

/* Prototipos de las funciones publicas */

void i2c_config(I2C_Handler_t *ptrHandlerI2C);
void i2c_startTransaction (I2C_Handler_t *ptrHandlerI2C);
void i2c_reStartTransaction (I2C_Handler_t *ptrHandlerI2C);
void i2c_sendSlaveAddressRW (I2C_Handler_t *ptrHandlerI2C, uint8_t slaveAddress, uint8_t readOrWrite);
void i2c_sendMemoryAddress (I2C_Handler_t *ptrHandlerI2C, uint8_t memAddr);
void i2c_sendDataByte (I2C_Handler_t *ptrHandlerI2C, uint8_t dataToWrite);
uint8_t i2c_readDataByte (I2C_Handler_t *ptrHandlerI2C);
void i2c_stopTransaction (I2C_Handler_t *ptrHandlerI2C);
void i2c_sendAck (I2C_Handler_t *ptrHandlerI2C);
void i2c_sendNoAck (I2C_Handler_t *ptrHandlerI2C);

uint8_t i2c_readSingleRegister (I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead);
void i2c_writeSingleRegister (I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead, uint8_t newValue);


#endif /* I2CDRIVER_H_ */

