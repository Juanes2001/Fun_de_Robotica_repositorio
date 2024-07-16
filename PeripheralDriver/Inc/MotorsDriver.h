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
	float    u_M1 ;
	float    u_M1_1 ;
	float    q_M1_0,q_M1_1,q_M1_2; // Constantes de control del motor 1
	float    e_M1,e_M1_1,e_M1_2;   // errores de control del motor 1
	uint16_t counts_M1;          // Cuentas netas del encoder motor 1

}Motor1_Config_t;

typedef struct
{
	float    u_M2 ;
	float    u_M2_1 ;
	float    q_M2_0,q_M2_1,q_M2_2; //Constantes de control del motor 2
	float    e_M2,e_M2_1,e_M2_2;   // Error de control del motor 2
	uint16_t counts_M2;			   // Cuentas netas del encoder motor 2

}Motor2_Config_t;

typedef struct
{
	Motor1_Config_t	   configM1;	// Configuracion del motor 1
	Motor2_Config_t	   configM2;	// Configuracion del motor 2

}Motor_Handler_t;


// Funciones

void setConstants(Motor_Handler_t *ptrMotorHandler, uint8_t motor, float k, float tau, float theta, uint16_t Ts);
double PID(Motor_Handler_t *ptrMotorHandler, float measure, uint8_t motor, float setpoint );
double map(double x, double in_min, double in_max, double out_min, double out_max);


#endif /* MOTORSDRIVER_H_ */
