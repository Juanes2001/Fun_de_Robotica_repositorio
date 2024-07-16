/*
 * MPUAccel.c
 *
 *  Created on: 26/11/2022
 *      Author: ALEJANDRA MARIA
 */

#include "MPUAccel.h"
#include "I2CDriver.h"

uint8_t i2cBuffer   = 0;  //almacena valores leidos en los registros del acelerometro
uint8_t AccelX_low  = 0;
uint8_t AccelX_high = 0;
int16_t AccelX      = 0;
uint8_t AccelY_low  = 0;
uint8_t AccelY_high = 0;
int16_t AccelY      = 0;
uint8_t GyroX_low   = 0;
uint8_t GyroX_high  = 0;
int16_t GyroX       = 0;
uint8_t GyroY_low   = 0;
uint8_t GyroY_high  = 0;
int16_t GyroY       = 0;

#define ADDRESS_UP   0b1101001
#define ADDRESS_DOWN 0b1101000

uint8_t readAccelValues(I2C_Handler_t *ptrHandlerI2C, uint8_t address){

	uint8_t auxRead = 0;

	i2c_startTransaction (ptrHandlerI2C);

	i2c_sendSlaveAddressRW (ptrHandlerI2C,ADDRESS_DOWN,0);

	i2c_sendMemoryAddress (ptrHandlerI2C, address);

    i2c_reStartTransaction (ptrHandlerI2C);

    i2c_sendSlaveAddressRW (ptrHandlerI2C,ADDRESS_DOWN,1);

    i2c_sendNoAck (ptrHandlerI2C);

    i2c_stopTransaction (ptrHandlerI2C);

    auxRead = i2c_readDataByte (ptrHandlerI2C);

    return auxRead;

}

void writeAccelValues(I2C_Handler_t *ptrHandlerI2C,uint8_t address, uint8_t newValue){

	i2c_startTransaction (ptrHandlerI2C);

	i2c_sendSlaveAddressRW (ptrHandlerI2C,ADDRESS_DOWN,0);

	i2c_sendMemoryAddress (ptrHandlerI2C, address);

	i2c_sendDataByte (ptrHandlerI2C, newValue);

	i2c_stopTransaction (ptrHandlerI2C);

}

int16_t X_Accel(I2C_Handler_t *ptrHandlerI2C){
	AccelX_low  = i2c_readSingleRegister(ptrHandlerI2C,ACCEL_XOUT_L);
	AccelX_high = i2c_readSingleRegister(ptrHandlerI2C,ACCEL_XOUT_H);
	AccelX = AccelX_high << 8 | AccelX_low;
	return AccelX;
}
int16_t Y_Accel(I2C_Handler_t *ptrHandlerI2C){

	AccelY_low = i2c_readSingleRegister(ptrHandlerI2C,ACCEL_YOUT_L);
	AccelY_high = i2c_readSingleRegister(ptrHandlerI2C,ACCEL_YOUT_H);
	AccelY = AccelY_high << 8 | AccelY_low;
	return AccelY;
}
int16_t X_Gyro(I2C_Handler_t *ptrHandlerI2C){
	GyroX_low  = i2c_readSingleRegister(ptrHandlerI2C,GYRO_XOUT_L);
	GyroX_high = i2c_readSingleRegister(ptrHandlerI2C,GYRO_XOUT_H);
	GyroX = GyroX_high << 8 | GyroX_low;
	return GyroX;
}
int16_t Y_Gyro(I2C_Handler_t *ptrHandlerI2C){
	GyroY_low  = i2c_readSingleRegister(ptrHandlerI2C,GYRO_YOUT_L);
	GyroY_high = i2c_readSingleRegister(ptrHandlerI2C,GYRO_YOUT_H);
	GyroY = GyroY_high << 8 | GyroY_low;
	return GyroY;
}


void startAccel(I2C_Handler_t *ptrHandlerI2C){
	writeAccelValues(ptrHandlerI2C, WHO_AM_I, ADDRESS_DOWN);
}
