/*
 * MotorsDriver.h
 *
 *  Created on: Apr 9, 2024
 *      Author: juan
 */

#ifndef MOTORSDRIVER_H_
#define MOTORSDRIVER_H_

#include "stm32f4xx.h"


typedef struct
{
	float    u; 			//Respuesta de salida
	float    e0;			//Error
	float    sum; 			//Suma integral
	float    ep;   			//Error previo
	float    kp,ki,kd;      // Constantes PID
}PID_Parameters_t;


typedef struct
{
	PID_Parameters_t *pid; // Puntero a estructura PID
	uint8_t counts;        // cuentas de cada motors
	float vel;				// velocidad de cada rueda
  	float dis;    			// distancia por muestreo

}Motor_Parameters_t;


// Funciones

void setConstants(Motor_Handler_t *ptrMotorHandler, uint8_t motor, float k, float tau, float theta, uint16_t Ts);
double PID(Motor_Handler_t *ptrMotorHandler, float measure, uint8_t motor, float setpoint );
double map(double x, double in_min, double in_max, double out_min, double out_max);


#endif /* MOTORSDRIVER_H_ */
