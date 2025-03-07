/*
 * AdcDriver.h
 *
 *  Created on: Month XX, 2022
 *      Author: namontoy
 */

#ifndef INC_ADCDRIVER_H_
#define INC_ADCDRIVER_H_

#include "stm32f4xx.h"
#include "GPIOxDriver.h"

#define ADC_CHANNEL_0		0
#define ADC_CHANNEL_1		1
#define ADC_CHANNEL_2		2
#define ADC_CHANNEL_3		3
#define ADC_CHANNEL_4		4
#define ADC_CHANNEL_5		5
#define ADC_CHANNEL_6		6
#define ADC_CHANNEL_7		7
#define ADC_CHANNEL_8		8
#define ADC_CHANNEL_9		9
#define ADC_CHANNEL_10		10
#define ADC_CHANNEL_11		11
#define ADC_CHANNEL_12		12
#define ADC_CHANNEL_13		13
#define ADC_CHANNEL_14		14
#define ADC_CHANNEL_15		15

#define ADC_RESOLUTION_12_BIT	0
#define ADC_RESOLUTION_10_BIT	1
#define ADC_RESOLUTION_8_BIT	2
#define ADC_RESOLUTION_6_BIT	3

#define ADC_ALIGNMENT_RIGHT		0
#define ADC_ALIGNMENT_LEFT		1

#define ADC_SAMPLING_PERIOD_3_CYCLES	0b000
#define ADC_SAMPLING_PERIOD_15_CYCLES	0b001
#define ADC_SAMPLING_PERIOD_28_CYCLES	0b010
#define ADC_SAMPLING_PERIOD_56_CYCLES	0b011
#define ADC_SAMPLING_PERIOD_84_CYCLES	0b100
#define ADC_SAMPLING_PERIOD_112_CYCLES	0b101
#define ADC_SAMPLING_PERIOD_144_CYCLES	0b110
#define ADC_SAMPLING_PERIOD_480_CYCLES	0b111

#define ADC_EVENT_DISABLE   0
#define ADC_EVENT_ENABLE    1

#define ADC_CONT_DISABLE 0
#define ADC_CONT_ENABLE  1

#define ADC_MULTCH_DISABLE 0
#define ADC_MULTCH_ENABLE  1

#define ADC_WATCHDOG_DISABLE 0
#define ADC_WATCHDOG_ENABLE  1



typedef struct
{
	uint8_t		channel;		  	 // Canal ADC que será utilizado para la conversión ADC
	uint8_t		resolution;		 	 // Precisión con la que el ADC hace la adquisición del dato
	uint16_t	samplingPeriod;	  	 // Tiempo deseado para hacer la adquisición del dato
	uint8_t		dataAlignment;	 	 // Alineación a la izquierda o a la derecha
	uint16_t	adcData;         	 // Dato de la conversión
	uint8_t     channelVector[16]; 	 // Vector de canales
	uint8_t     continuosModeEnable; // Modo de conversion continua
	uint8_t 	multiChannel;        // Activa el multicanal o la conversion sigle channel.
	uint8_t     numeroDeCanales;	 // Cantidad de canales a convertir
	uint8_t 	watchdogs_Enable;    // Activamos o desactivamos el modo de WatchDosgs
	uint16_t 	threshold_up;		 // Limite superior del watchdog
	uint16_t 	threshold_down;		 // Limite inferior del watchdog
}ADC_Config_t;

enum {
	e_ADC_PRIORITY_6 =6,
	e_ADC_PRIORITY_7,
	e_ADC_PRIORITY_8,
	e_ADC_PRIORITY_9,
	e_ADC_PRIORITY_10,
	e_ADC_PRIORITY_11,
	e_ADC_PRIORITY_12,
	e_ADC_PRIORITY_13,
	e_ADC_PRIORITY_14,
	e_ADC_PRIORITY_15
};

void adc_Config(ADC_Config_t *adcConfig);

void adc_ON(void);
void adc_OFF(void);
void adc_CONT_ON(void);
void adc_CONT_OFF(void);
void adcComplete_Callback(void);
void watchdogs_Callback(void);
void startConvertion(void);
uint16_t getADC(void);
void adcExternalEXTIConfig ();
void adcTimerEventConfig();
void adc_Set_Priority(ADC_Config_t *ptrAdcConfig, uint8_t newPriority);

void ADC_IRQHandler(void);

#endif /* INC_ADCDRIVER_H_ */
