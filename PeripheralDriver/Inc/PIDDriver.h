/*
 * PIDDriver.h
 *
 *  Created on: 1 dic. 2022
 *      Author: JUAN ESTEBAN
 */

#ifndef PIDDRIVER_H_
#define PIDDRIVER_H_

#include <stm32f4xx.h>
#include "SysTickDriver.h"

#define PID_CD_DIRECT  0
#define PID_CD_REVERSE 1

typedef struct
{

	uint8_t   ControllerDirection; //define la dirección del motor

	uint32_t        SampleTime; //Tiempo de muestreo

	double          DispKp; //Kp lista para actualizarse
	double          DispKi; //Ki lista para actualizarse
	double          DispKd; //Kd lista para actualizarse

	double          Kp; //Kp para los calculos
	double          Ki; //Ki para los calculos
	double          Kd; //Kd para los calculos


	double          Setpoint; //punto de referencia al que debe llegar el sistema.

	double          OutputSum; //  variable que muestra la Sumatoria realizada para la la constante integrativa
	double          LastInput; // variable que registra el input del ciclo anterior



	double          OutMin; //valor mínimo de la variable de control (duty)
	double          OutMax; //valor máximo de la variable de control (duty)

}PID_Config_t;



void PID_init(PID_Config_t *PID);
double Compute_PID( PID_Config_t *PID, double input);
void PID_SetControllerDirection(PID_Config_t *PID, uint8_t Direction);
uint8_t PID_GetDirection(PID_Config_t *PID);
void PID_SetTunings(PID_Config_t *PID, double Kp, double Ki, double Kd);
void PID_SetSampleTime(PID_Config_t *PID, uint32_t NewSampleTime);
void PID_SetKp(PID_Config_t *PID, double Kp);
void PID_SetKi(PID_Config_t *PID, double Ki);
void PID_SetKd(PID_Config_t *PID, double Kd);
double PID_GetKp(PID_Config_t *PID);
double PID_GetKi(PID_Config_t *PID);
double PID_GetKd(PID_Config_t *PID);


#endif /* PIDDRIVER_H_ */
