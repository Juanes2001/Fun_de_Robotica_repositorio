/*
 * PIDDriver.c
 *
 *  Created on: 1 dic. 2022
 *      Author: JUAN ESTEBAN
 */


#include "PIDDriver.h"
#include "SysTickDriver.h"

/*funcion que inicializa las constantes del PID y la configuración de dirección*/
void PID_init(PID_Config_t *PID){
	PID_SetControllerDirection(PID, PID->ControllerDirection);
	PID_SetTunings(PID, PID->Kp, PID->Ki, PID->Kd);

}

/*funcion que se encarga de realizar y actualizar  los cálculos de las constantes */
double  Compute_PID( PID_Config_t *PID, double input){

	double error; //diferencia entre el setpoint y los valores reales del acelerometro
	double dInput; //cambio pequeño entre entradas de aceleracion
	double output; //respuesta de control (duty).

	error = PID->Setpoint - input;
	dInput  = (input - PID->LastInput);

	PID->OutputSum += (PID->Ki * error); //definición de la sumatoria para el valor integrativo

	/* condicion de saturación (puntos extremos) */

	if (PID->OutputSum > PID->OutMax)
	{
		PID->OutputSum = PID->OutMax;
	}
	else if (PID->OutputSum < PID->OutMin)
	{
		PID->OutputSum = PID->OutMin;
	}
	else {

	}

	// respuesta de control en términos únicamente de al constante proporcional

	output = PID->Kp * error;

	/*Respuesta de control en términos de la constante derivativa */
	output += PID->OutputSum - PID->Kd * dInput;

	//Condición de saturación para Kd.

	if (output > PID->OutMax)
	{
		output = PID->OutMax;
	}
	else if (output < PID->OutMin)
	{
		output = PID->OutMin;
	}
	else {

	}
	PID->LastInput = input;
	return output;

}
/*función que cambia la dirección de la acción de control (giro motor) en caso de obtener aceleración de ambos signos (+-16000)*/
void PID_SetControllerDirection(PID_Config_t *PID, uint8_t Direction){

		if ( Direction !=PID->ControllerDirection)
		{

			PID->Kp = (0 - PID->Kp);
			PID->Ki = (0 - PID->Ki);
			PID->Kd = (0 - PID->Kd);

		}

		PID->ControllerDirection = Direction;
}
/*función para obtener la dirección y actualizarla*/
uint8_t PID_GetDirection(PID_Config_t *PID){
	return PID->ControllerDirection;
}
/*Función para poder acceder a los valores de las constantes*/
void PID_SetTunings(PID_Config_t *PID, double Kp, double Ki, double Kd){
	double SampleTimeInSec;


		if (Kp < 0 || Ki < 0 || Kd < 0)
		{
			return;
		}




		PID->DispKp = Kp;
		PID->DispKi = Ki;
		PID->DispKd = Kd;

		/* ~~~~~~~~~ Calculate time ~~~~~~~~ */
		SampleTimeInSec = ((double)PID->SampleTime) / 1000;

		PID->Kp = Kp;
		PID->Ki = Ki * SampleTimeInSec; //Revisar esta operacion
		PID->Kd = Kd / SampleTimeInSec;

		/* ~~~~~~~~ Check direction ~~~~~~~~ */
		if (PID->ControllerDirection == PID_CD_REVERSE)
		{

			PID->Kp = (0 - PID->Kp);
			PID->Ki = (0 - PID->Ki);
			PID->Kd = (0 - PID->Kd);

		}
}

/*función que define el tiempo de muestreo y cada cuando se ejerce la acción de control*/
void PID_SetSampleTime(PID_Config_t *PID, uint32_t NewSampleTime){
	double ratio;

	if (NewSampleTime > 0)
	{

		ratio = (double)NewSampleTime / (double)PID->SampleTime;

		PID->Ki *= ratio;
		PID->Kd /= ratio;
		PID->SampleTime = (uint32_t)NewSampleTime;

	}
}

void PID_SetKp(PID_Config_t *PID, double Kp){

	PID->DispKp = Kp;


	PID->Kp = Kp;

	/* ~~~ Check direction ~~~ */
	if (PID->ControllerDirection == PID_CD_REVERSE)
	{
		PID->Kp = (0 - PID->Kp);
	}

}


void PID_SetKi(PID_Config_t *PID, double Ki){

	double SampleTimeInSec;

	PID->DispKi = Ki;

	/* ~~~ Calculate time ~~~~ */
	SampleTimeInSec = ((double)PID->SampleTime) / 1000;

	PID->Ki = Ki * SampleTimeInSec;

	/* ~~~ Check direction ~~~ */
	if (PID->ControllerDirection == PID_CD_REVERSE)
	{

		PID->Ki = (0 - PID->Ki);

	}
}

void PID_SetKd(PID_Config_t *PID, double Kd){

	double SampleTimeInSec;

	PID->DispKd = Kd;

	/* ~~~ Calculate time ~~~~ */
	SampleTimeInSec = ((double)PID->SampleTime) / 1000;

	PID->Kd = Kd / SampleTimeInSec;

	/* ~~~ Check direction ~~~ */
	if (PID->ControllerDirection == PID_CD_REVERSE)
	{

		PID->Kd = (0 - PID->Kd);

	}
}

/*funciones para recoger los valores de las constantes y poder usarlos*/
double PID_GetKp(PID_Config_t *PID)
{
	return PID->DispKp;
}
double PID_GetKi(PID_Config_t *PID)
{
	return PID->DispKi;
}
double PID_GetKd(PID_Config_t *PID)
{
	return PID->DispKd;
}

