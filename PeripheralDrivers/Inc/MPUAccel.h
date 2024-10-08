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
#include "GPIOxDriver.h"
#include "stdlib.h"


/* Definiciones para funcionamiento del acelerometro */
#define ADDRESS_UP          0b1101001
#define ADDRESS_DOWN        0b1101000

//Escalas para cada una de las cantidades

//Diferentes escalas para la aceleracion
#define ACCEL_2G  0
#define ACCEL_4G  1
#define ACCEL_8G  2
#define ACCEL_16G 3

//Sensibilidades respectivamente por rango de aceleración

#define ACCEL_2G_SENS  16384
#define ACCEL_4G_SENS  8192
#define ACCEL_8G_SENS  4096
#define ACCEL_16G_SENS 2048

//Diferentes escalas para el GYRO

#define GYRO_250   0
#define GYRO_500   1
#define GYRO_1000  2
#define GYRO_2000  3

//Sensibilidades respectivamente por rango de GYRO

#define GYRO_250_SENS  131
#define GYRO_500_SENS  66
#define GYRO_1000_SENS 33
#define GYRO_2000_SENS 16

// TIPO DE EJE A CALIBRAR
#define CALIB_X 0
#define CALIB_Y 1
#define CALIB_Z 2


//Ejes

//Eje de aceleracion en X,Y,Z
#define ACCEL_X 0
#define ACCEL_Y 1
#define ACCEL_Z 2

//Eje de rotacion en X,Y,Z
#define GYRO_X 0
#define GYRO_Y 1
#define GYRO_Z 2


typedef struct
{
	uint8_t 		fullScaleACCEL;
	uint8_t 		fullScaleGYRO;
	GPIO_Handler_t  *ptrGPIOhandlerSCL;
	GPIO_Handler_t  *ptrGPIOhandlerSDA;
	I2C_Handler_t   *ptrI2Chandler;

}MPUAccel_Config;


//Funciones

void configMPUAccel (MPUAccel_Config *ptrMPUAccel);
void writeData (MPUAccel_Config *ptrMPUAccel, uint8_t RA, uint8_t data);
uint8_t readData (MPUAccel_Config *ptrMPUAccel, uint8_t RA);
int* readContinuousData (MPUAccel_Config *ptrMPUAccel, uint8_t RA, uint8_t numbR_to_Read);


float readAccel_X (MPUAccel_Config *ptrMPUAccel);
float readAccel_Y (MPUAccel_Config *ptrMPUAccel);
float readAccel_Z (MPUAccel_Config *ptrMPUAccel);


float readGyro_X (MPUAccel_Config *ptrMPUAccel);
float readGyro_Y (MPUAccel_Config *ptrMPUAccel);
float readGyro_Z (MPUAccel_Config *ptrMPUAccel);

uint8_t WHOIAM (MPUAccel_Config *ptrMPUAccel);

//float toFloat(CustomFloat32 cf);

#endif /* MPUACCEL_H_ */
