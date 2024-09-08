/*
 * MotorsDriver.c
 *
 *  Created on: Apr 9, 2024
 *      Author: juan
 */

#include "MotorsDriver.h"


//Funcion para actualizar el dutty del pwm
void updateDuttyMotor(Motor_Handler_t *ptrMotorHandler,  float newValue)
{
		//Guardar valor del dutty
		ptrMotorHandler->configMotor.dutty = newValue;
		//Actualizamos el valor del dutty
		updateDuttyCycleAfOpt(ptrMotorHandler->phandlerPWM, ptrMotorHandler->configMotor.dutty);
}


void updateFrequencyMotor(Motor_Handler_t *ptrMotorHandler, BasicTimer_Handler_t *ptrBTimerHandler, uint16_t newValue)
{
		//Guardar valor del dutty
		*(ptrMotorHandler->configMotor.frecuency) = newValue;
		//Actualizamos el valor del frecuencia y dutty
		updateFrequency(ptrMotorHandler->phandlerPWM, *(ptrMotorHandler->configMotor.frecuency));
		updateDuttyCycleAfOpt(ptrMotorHandler->phandlerPWM, ptrMotorHandler->configMotor.dutty);
}

void updateDirMotor(Motor_Handler_t *ptrMotorHandler)
{
		//Cambiar el valor de la direccion
		ptrMotorHandler->configMotor.dir = (~ptrMotorHandler->configMotor.dir)&(0x01);
		//Cambiamos la direccion del motor
		GPIO_WritePin(ptrMotorHandler->phandlerGPIOIN, ptrMotorHandler->configMotor.dir&SET);
		PWMx_Toggle(ptrMotorHandler->phandlerPWM);
}

double map(double x, double in_min, double in_max, double out_min, double out_max){

	double primera = (x - in_min);
	double segundo = (out_max - out_min);
	double tercero = (in_max - in_min);

	double result = (primera * segundo / tercero ) + out_min;

	return  result;
}





