/*
 * MotorsDriver.c
 *
 *  Created on: Apr 9, 2024
 *      Author: juan
 */

#include "MotorsDriver.h"
#include "PwmDriver.h"


void setConstants(Motor_Handler_t *ptrMotorHandler, uint8_t motor , float k, float tau, float theta, uint16_t Ts){

	float kp   = 0;
	float taoi = 0;
	float taod = 0;

	kp   = (1.2*tau)/(k*theta);
	taoi = 2*theta;
	taod = 0.5*theta;

	switch (motor) {

		case 1:{

			ptrMotorHandler->configM1.q_M1_0 = kp*(1+Ts/(2.0*taoi)+taod/Ts);
			ptrMotorHandler->configM1.q_M1_1 = -kp*(1-Ts/(2.0*taoi)+(2.0*taod)/Ts);
			ptrMotorHandler->configM1.q_M1_2 = (kp*taod)/Ts;

			break;
		}case 2:{

			ptrMotorHandler->configM2.q_M2_0 = kp*(1+Ts/(2.0*taoi)+taod/Ts);
			ptrMotorHandler->configM2.q_M2_1 = -kp*(1-Ts/(2.0*taoi)+(2.0*taod)/Ts);
			ptrMotorHandler->configM2.q_M2_2 = (kp*taod)/Ts;

			break;
		}default :{

			__NOP();
			break;
		}

	}

}

double PID(Motor_Handler_t *ptrMotorHandler, float measure , uint8_t motor, float setpoint ){

	float u,p_0,p_1,p_2;


	switch (motor) {

		case 1:{


			ptrMotorHandler->configM1.e_M1 = setpoint - measure;

			p_0 = ptrMotorHandler->configM1.q_M1_0 * ptrMotorHandler->configM1.e_M1;
			p_1 = ptrMotorHandler->configM1.q_M1_1 * ptrMotorHandler->configM1.e_M1_1;
			p_2 = ptrMotorHandler->configM1.q_M1_2 * ptrMotorHandler->configM1.e_M1_2;

			ptrMotorHandler->configM1.u_M1 = ptrMotorHandler->configM1.u_M1_1 + p_0 + p_1 + p_2;
			u = ptrMotorHandler->configM1.u_M1;

			// Actualizamos los datos

			ptrMotorHandler->configM1.e_M1_1 = ptrMotorHandler->configM1.e_M1;
			ptrMotorHandler->configM1.e_M1_1 = ptrMotorHandler->configM1.e_M1_2;
			ptrMotorHandler->configM1.u_M1_1 = ptrMotorHandler->configM1.u_M1;


			break;
		}case 2:{

			ptrMotorHandler->configM2.e_M2 = setpoint - measure;

			p_0 = ptrMotorHandler->configM2.q_M2_0 * ptrMotorHandler->configM2.e_M2;
			p_1 = ptrMotorHandler->configM2.q_M2_1 * ptrMotorHandler->configM2.e_M2_1;
			p_2 = ptrMotorHandler->configM2.q_M2_2 * ptrMotorHandler->configM2.e_M2_2;

			ptrMotorHandler->configM2.u_M2 = ptrMotorHandler->configM2.u_M2_1 + p_0 + p_1 + p_2;
			u = ptrMotorHandler->configM2.u_M2;

			// Actualizamos los datos

			ptrMotorHandler->configM2.e_M2_1 = ptrMotorHandler->configM2.e_M2;
			ptrMotorHandler->configM2.e_M2_1 = ptrMotorHandler->configM2.e_M2_2;
			ptrMotorHandler->configM2.u_M2_1 = ptrMotorHandler->configM2.u_M2;


			break;
		}default :{

			__NOP();
			break;
		}

	}

	return u;

}

double map(double x, double in_min, double in_max, double out_min, double out_max){

	double primera = (x - in_min);
	double segundo = (out_max - out_min);
	double tercero = (in_max - in_min);

	double result = (primera * segundo / tercero ) + out_min;

	return  result;
}





