/*
 * CaptureFreqDriver.h
 *
 *  Created on: 19 nov. 2022
 *      Author: JUAN ESTEBAN
 */

#ifndef CAPTUREFREQDRIVER_H_
#define CAPTUREFREQDRIVER_H_


#include "stm32f4xx.h"
#include <stdlib.h>

#define CAPTURE_CHANNEL_1   0
#define CAPTURE_CHANNEL_2   1
#define CAPTURE_CHANNEL_3   2
#define CAPTURE_CHANNEL_4   3

#define CAPTURE_PREESCALER_1_1 0
#define CAPTURE_PREESCALER_2_1 1
#define CAPTURE_PREESCALER_4_1 2
#define CAPTURE_PREESCALER_8_1 3

#define CAPTURE_TIMER_SPEED_1us    16
#define CAPTURE_TIMER_SPEED_10us   160
#define CAPTURE_TIMER_SPEED_100us  1600
#define CAPTURE_TIMER_SPEED_1ms    16000

#define CAPTURE_FALLING_EDGE 0
#define CAPTURE_RISING_EDGE 1

/**/
typedef struct
{
	uint8_t  channel;          //Canal PWM relacionado con el timer
	uint8_t  edgeSignal;       //Seleccionamos el tipo de flanco que se desea capurar
	uint32_t timerSpeed;       //A que velocidad se incrementa el Timer
	uint32_t prescalerCapture; //A que velocidad se incrementaa el Timer
}Capture_Config_t;

/**/
typedef struct
{
	TIM_TypeDef       *ptrTIMx; // Timer al que esta asociado la captura
	Capture_Config_t  config;   // Configuracion inicial del PWM
}Capture_Handler_t;

/*Prototipos de las funciones*/
void capture_Config (Capture_Handler_t *ptrCaptureHandler);
void startCapture (Capture_Handler_t *ptrCaptureHandler);
void stopCapture (Capture_Handler_t *ptrCaptureHandler);
uint32_t timeStamp(Capture_Handler_t  *ptrCaptureHandler);
uint32_t getFreq(Capture_Handler_t  *ptrCaptureHandler, uint32_t ts1, uint32_t ts2 );
void clean(Capture_Handler_t *ptrCaptureHandler);


#endif /* CAPTUREFREQDRIVER_H_ */
