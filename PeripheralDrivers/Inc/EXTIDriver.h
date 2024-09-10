/*
 * ExtiDriver.h
 *
 *  Created on: May 10, 2022
 *      Author: namontoy
 */

#ifndef EXTIDRIVER_H_
#define EXTIDRIVER_H_

#include "stm32f4xx.h"
#include "GPIOxDriver.h"

#define EXTERNAL_INTERRUPT_FALLING_EDGE	 	     0
#define EXTERNAL_INTERRUPT_RISING_EDGE		     1
#define EXTERNAL_INTERRUPT_RASINGANDFALLING_EDGE 2

typedef struct
{
	GPIO_Handler_t *pGPIOHandler;	// Canal ADC que será utilizado para la conversión ADC
	uint8_t			edgeType;		// Se selecciona si se desea un tipo de flanco subiendo o bajando
	uint8_t         priority;
}EXTI_Config_t;

enum
{
	e_EXTI_PRIORITY_6 = 6,
	e_EXTI_PRIORITY_7,
	e_EXTI_PRIORITY_8,
	e_EXTI_PRIORITY_9,
	e_EXTI_PRIORITY_10,
	e_EXTI_PRIORITY_11,
	e_EXTI_PRIORITY_12,
	e_EXTI_PRIORITY_13,
	e_EXTI_PRIORITY_14,
	e_EXTI_PRIORITY_15
};


void extInt_Config(EXTI_Config_t *extiConfig);
void configExternalTrigger(GPIO_Handler_t *pGPIOHandler);
void exti_Set_Priority (EXTI_Config_t *extiConfig, uint8_t newPriority);


void callback_extInt0(void);
void callback_extInt1(void);
void callback_extInt2(void);
void callback_extInt3(void);
void callback_extInt4(void);
void callback_extInt5(void);
void callback_extInt6(void);
void callback_extInt7(void);
void callback_extInt8(void);
void callback_extInt9(void);
void callback_extInt10(void);
void callback_extInt11(void);
void callback_extInt12(void);
void callback_extInt13(void);
void callback_extInt14(void);
void callback_extInt15(void);



void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

#endif /* EXTIDRIVER_H_ */
