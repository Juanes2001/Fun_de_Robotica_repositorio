/*
 * BasicTimer.h
 *
 *  Created on: Apr 18, 2022
 *      Author: Juan Esteban Rodriguez Ochoa
 */

#ifndef INC_BASICTIMER_H_
#define INC_BASICTIMER_H_

#include "stm32f4xx.h"


#define BTIMER_MODE_UP		0
#define BTIMER_MODE_DOWN	1

#define BTIMER_SPEED_16MHz_10us	    160
#define BTIMER_SPEED_16MHz_100us    1600
#define BTIMER_SPEED_16MHz_1ms	    16000

#define BTIMER_SPEED_20MHz_10us	    200
#define BTIMER_SPEED_20MHz_100us    2000
#define BTIMER_SPEED_20MHz_1ms	    20000

#define BTIMER_SPEED_30MHz_10us	    300
#define BTIMER_SPEED_30MHz_100us    3000
#define BTIMER_SPEED_30MHz_1ms	    30000

#define BTIMER_SPEED_40MHz_10us	    400
#define BTIMER_SPEED_40MHz_100us    4000
#define BTIMER_SPEED_40MHz_1ms	    40000

#define BTIMER_SPEED_50MHz_10us	    500
#define BTIMER_SPEED_50MHz_100us    5000
#define BTIMER_SPEED_50MHz_1ms	    50000

#define BTIMER_SPEED_60MHz_10us	    600
#define BTIMER_SPEED_60MHz_100us    6000
#define BTIMER_SPEED_60MHz_1ms	    60000

#define BTIMER_SPEED_70MHz_10us	    700
#define BTIMER_SPEED_70MHz_100us    7000
#define BTIMER_SPEED_70MHz_1ms	    70000

#define BTIMER_SPEED_80MHz_10us	    800
#define BTIMER_SPEED_80MHz_100us    8000
#define BTIMER_SPEED_80MHz_1ms	    80000

#define BTIMER_SPEED_90MHz_10us	    900
#define BTIMER_SPEED_90MHz_100us    9000
#define BTIMER_SPEED_90MHz_1ms	    90000

#define BTIMER_SPEED_100MHz_10ns	1
#define BTIMER_SPEED_100MHz_10us	1000
#define BTIMER_SPEED_100MHz_100us	10000
#define BTIMER_SPEED_100MHz_1ms	    100000

#define BTIMER_DISABLE_INTERRUPT 0
#define BTIMER_ENABLE_INTERRUPT  1


/* Estructura que contiene la configuración mínima necesaria para el manejo del Timer.*/
typedef struct
{
	uint8_t		TIMx_mode; 		// Up or dowm
	uint32_t	TIMx_speed;		// A qué velocidad se incrementa el Timer
	uint32_t	TIMx_period;	// Valor en ms del periodo del Timer
	uint8_t		TIMx_interruptEnable;	// Activa o desactiva el modo interrupción del timer.
}BasicTimer_Config_t;

/* Handler para el Timer*/
typedef struct
{
	TIM_TypeDef			*ptrTIMx;
	BasicTimer_Config_t	TIMx_Config;
}BasicTimer_Handler_t;

void BasicTimer_Config(BasicTimer_Handler_t *ptrBTimerHandler);

void BasicTimer1_Callback(void);
void BasicTimer2_Callback(void); /* Esta función debe ser sobre-escrita en el main para que el sistema funcione*/
void BasicTimer3_Callback(void);
void BasicTimer4_Callback(void);
void BasicTimer5_Callback(void);

void Capture_TIM2_Ch1_Callback(void);
void Capture_TIM2_Ch2_Callback(void);
void Capture_TIM2_Ch3_Callback(void);
void Capture_TIM2_Ch4_Callback(void);

void Capture_TIM3_Ch1_Callback(void);
void Capture_TIM3_Ch2_Callback(void);
void Capture_TIM3_Ch3_Callback(void);
void Capture_TIM3_Ch4_Callback(void);

void Capture_TIM4_Ch1_Callback(void);
void Capture_TIM4_Ch2_Callback(void);
void Capture_TIM4_Ch3_Callback(void);
void Capture_TIM4_Ch4_Callback(void);

void Capture_TIM5_Ch1_Callback(void);
void Capture_TIM5_Ch2_Callback(void);
void Capture_TIM5_Ch3_Callback(void);
void Capture_TIM5_Ch4_Callback(void);

void startTimer (BasicTimer_Handler_t *ptrTimerConfig);
void stopTimer (BasicTimer_Handler_t *ptrTimerConfig);
#endif /* INC_BASICTIMER_H_ */
