/*
 * MPUAccel.h
 *
 *  Created on: 26/11/2022
 *      Author: ALEJANDRA MARIA
 */

#ifndef MPUACCEL_H_
#define MPUACCEL_H_

#include <stm32f4xx.h>
#include "I2CDriver.h"



/* Definiciones para funcionamiento del acelerometro */
#define ADDRESS_UP          0b1101001
#define ADDRESS_DOWN        0b1101000
#define ACCEL_XOUT_H 		59
#define ACCEL_XOUT_L		60
#define ACCEL_YOUT_H		61
#define ACCEL_YOUT_L		62
#define ACCEL_ZOUT_H		63
#define ACCEL_ZOUT_L		64

#define GYRO_XOUT_H 		67
#define GYRO_XOUT_L			68
#define GYRO_YOUT_H			69
#define GYRO_YOUT_L			70
#define GYRO_ZOUT_H			71
#define GYRO_ZOUT_L			72

#define PWR_MGMT_l			107
#define WHO_AM_I			117

void writeAccelValues(I2C_Handler_t *ptrHandlerI2C, uint8_t address , uint8_t newValue);
uint8_t readAccelValues(I2C_Handler_t *ptrHandlerI2C, uint8_t address);
void startAccel(I2C_Handler_t *ptrHandlerI2C);
int16_t X_Accel(I2C_Handler_t *ptrHandlerI2C);
int16_t Y_Accel(I2C_Handler_t *ptrHandlerI2C);
int16_t X_Gyro(I2C_Handler_t *ptrHandlerI2C);
int16_t Y_Gyro(I2C_Handler_t *ptrHandlerI2C);


#endif /* MPUACCEL_H_ */
