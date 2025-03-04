/*
 * PwmDriver.h
 *
 *  Created on: May 14, 2022
 *      Author: namontoy
 */

#ifndef PWMDRIVER_H_
#define PWMDRIVER_H_

#include "stm32f4xx.h"

#define PWM_CHANNEL_1	0
#define PWM_CHANNEL_2	1
#define PWM_CHANNEL_3	2
#define PWM_CHANNEL_4	3

#define PWM_IN_CHANNEL_1	0
#define PWM_IN_CHANNEL_2	1



#define PWM_SPEED_16MHz_1us	    16
#define PWM_SPEED_16MHz_10us	160
#define PWM_SPEED_16MHz_100us	1600
#define PWM_SPEED_16MHz_1ms	    16000

#define PWM_SPEED_20MHz_1us	    20
#define PWM_SPEED_20MHz_10us	200
#define PWM_SPEED_20MHz_100us	2000
#define PWM_SPEED_20MHz_1ms	    20000

#define PWM_SPEED_30MHz_1us	    30
#define PWM_SPEED_30MHz_10us	300
#define PWM_SPEED_30MHz_100us	3000
#define PWM_SPEED_30MHz_1ms	    30000

#define PWM_SPEED_40MHz_1us	    40
#define PWM_SPEED_40MHz_10us	400
#define PWM_SPEED_40MHz_100us	4000
#define PWM_SPEED_40MHz_1ms	    40000

#define PWM_SPEED_50MHz_1us	    50
#define PWM_SPEED_50MHz_10us	500
#define PWM_SPEED_50MHz_100us	5000
#define PWM_SPEED_50MHz_1ms	    50000

#define PWM_SPEED_60MHz_1us	    60
#define PWM_SPEED_60MHz_10us	600
#define PWM_SPEED_60MHz_100us	6000
#define PWM_SPEED_60MHz_1ms	    60000

#define PWM_SPEED_70MHz_1us	    70
#define PWM_SPEED_70MHz_10us	700
#define PWM_SPEED_70MHz_100us	7000
#define PWM_SPEED_70MHz_1ms	    70000

#define PWM_SPEED_80MHz_1us	    80
#define PWM_SPEED_80MHz_10us	800
#define PWM_SPEED_80MHz_100us	8000
#define PWM_SPEED_80MHz_1ms	    80000

#define PWM_SPEED_90MHz_1us	    90
#define PWM_SPEED_90MHz_10us	900
#define PWM_SPEED_90MHz_100us	9000
#define PWM_SPEED_90MHz_1ms	    90000

#define PWM_SPEED_100MHz_10ns   1
#define PWM_SPEED_100MHz_1us    100
#define PWM_SPEED_100MHz_10us   1000
#define PWM_SPEED_100MHz_100us  10000
#define PWM_SPEED_100MHz_1ms	100000


#define PWM_ENABLE_POLARITY      1
#define PWM_DISABLE_POLARITY     0

#define PWM_ENABLE_OPTOCOUPLER   1
#define PWM_DISABLE_OPTOCOUPLER  0

#define PWM_ENABLE_ONE_PULSE   1
#define PWM_DISABLE_ONE_PULSE  0


/**/
typedef struct
{
	uint8_t		channel; 		// Canal PWM relacionado con el TIMER
	uint32_t	prescaler;		// A qué velocidad se incrementa el Timer
	double		periodo;		// Indica el número de veces que el Timer se incrementa, el periodo de la frecuencia viene dado por Time_Fosc * PSC * ARR
	float   	duttyCicle;		// Valor en porcentaje (%) del tiempo que la señal está en alto
	uint8_t     polarity;       // Se activa el modo de polaridad o no
	uint8_t     optocoupler;    // Se activa la opcion de optoacolpador donde se cambia toda la propiedad del dutty
	uint8_t     one_pulse;      // Se activa o se desactiva la opcion de un pulso
	uint8_t     channel_in;		// Seleccionamos el canal en el que se hara el trigger de entrada
}PWM_Config_t;

/**/
typedef struct
{
	TIM_TypeDef		*ptrTIMx;	// Timer al que esta asociado el PWM
	PWM_Config_t	config;	    // Configuración inicial del PWM
}PWM_Handler_t;

extern uint16_t periodo;


/* Prototipos de las funciones */
void pwm_Config(PWM_Handler_t *ptrPwmHandler);
void setFrequency(PWM_Handler_t *ptrPwmHandler);
void updateFrequency(PWM_Handler_t *ptrPwmHandler, uint16_t freq);
void setDuttyCycle(PWM_Handler_t *ptrPwmHandler);
void setDuttyCycleAfOpt(PWM_Handler_t *ptrPwmHandler);
void updateDuttyCycle(PWM_Handler_t *ptrPwmHandler, float newDutty);
void updateDuttyCycleAfOpt(PWM_Handler_t *ptrPwmHandler, float newDutty);
uint8_t enableOutput(PWM_Handler_t *ptrPwmHandler);
uint8_t disableOutput(PWM_Handler_t *ptrPwmHandler);
void startPwmSignal(PWM_Handler_t *ptrPwmHandler);
void stopPwmSignal(PWM_Handler_t *ptrPwmHandler);
void enableEvent(PWM_Handler_t *ptrPwmHandler);
void disableEvent(PWM_Handler_t *ptrPwmHandler);
void PWMx_Toggle(PWM_Handler_t *ptrPwmHandler);
uint8_t enableComplementaryOutput(PWM_Handler_t *ptrPwmHandler);
void outputSelector(PWM_Handler_t *ptrPwmHandler);

uint8_t showPWM (PWM_Handler_t *ptrPwmHandler);
uint8_t showPWMBfOpt (PWM_Handler_t *ptrPwmHandler);

#endif /* PWMDRIVER_H_ */
