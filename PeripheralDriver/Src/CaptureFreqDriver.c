/*
 * CaptureFreqDriver.c
 *
 *  Created on: 19 nov. 2022
 *      Author: JUAN ESTEBAN
 */

#include "CaptureFreqDriver.h"
#include <stdlib.h>


void capture_Config (Capture_Handler_t *ptrCaptureHandler){
		//Actuvamos el timer al que se le usaraan sus canales

	__disable_irq();

	if (ptrCaptureHandler->ptrTIMx == TIM2){

		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	}else if (ptrCaptureHandler->ptrTIMx == TIM3){

		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	}else if (ptrCaptureHandler->ptrTIMx == TIM4){

		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	}else if (ptrCaptureHandler->ptrTIMx == TIM5){

		RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

	}else {
		__NOP();
	}

	/*
	 * Estamos en UP Mode, el limite se carga en ARR y se comienza el
	 * CNT en 0
	 */
	ptrCaptureHandler->ptrTIMx->CR1 &= ~TIM_CR1_DIR;

	/*
	 * Configuramos los bits CCxS del registro TIMy_CCMR1, de forma que sea modo entrada
	 * (Para cada canal hay un conjunto CCxS)
	 *
	 * Ademas, en el mismo "case" podemos configurar el modo del PWM, su polaridad
	 *
	 * Y ademas activamos el preload bit, para que cada vez exista un update-event
	 * el valor carhgado en el CCRx sera recargado en el registro Shadow del PWM
	 */

	//Ademas activamos las interrupciones por conteo de tiempo en flanco
	switch (ptrCaptureHandler->config.channel) {
		case CAPTURE_CHANNEL_1:
			//borramos el posible valor cargado
			ptrCaptureHandler->ptrTIMx->CCMR1 &= ~(TIM_CCMR1_CC1S);
			//Configuramos como entrada en el mismo canal
			ptrCaptureHandler->ptrTIMx->CCMR1 = (1 << TIM_CCMR1_CC1S_Pos);

			//configuramos el filtro (sin filtro)
			ptrCaptureHandler->ptrTIMx->CCMR1 &= ~(0xF << TIM_CCMR1_IC1F_Pos);
			ptrCaptureHandler->ptrTIMx->CCMR1 |= (0x3 << TIM_CCMR1_IC1F_Pos);

			//configuramos el preescaler
			ptrCaptureHandler->ptrTIMx->CCMR1 &= ~(0x3 << TIM_CCMR1_IC1PSC_Pos);
			ptrCaptureHandler->ptrTIMx->CCMR1 |= (ptrCaptureHandler->config.prescalerCapture << TIM_CCMR1_IC1PSC_Pos);

			// Configuramos el flanco que deseamos capturar
			if (ptrCaptureHandler->config.edgeSignal == CAPTURE_RISING_EDGE){
				//Configuramos 00, para rising edge
				ptrCaptureHandler->ptrTIMx->CCER &= ~(TIM_CCER_CC1P);
				ptrCaptureHandler->ptrTIMx->CCER &= ~(TIM_CCER_CC1NP);
			}else{
				//Configuramos 01, para falling edge
				ptrCaptureHandler->ptrTIMx->CCER |= (TIM_CCER_CC1P);
				ptrCaptureHandler->ptrTIMx->CCER &= ~(TIM_CCER_CC1NP);
			}

			//Ademas activamos las interrupciones por conteo de tiempo en flanco
			ptrCaptureHandler->ptrTIMx->DIER |= TIM_DIER_CC1IE;

			break;
		case CAPTURE_CHANNEL_2:
			//borramos el posible valor cargado
			ptrCaptureHandler->ptrTIMx->CCMR1 &= ~(TIM_CCMR1_CC2S);
			//Configuramos como entrada en el mismo canal
			ptrCaptureHandler->ptrTIMx->CCMR1 = (1 << TIM_CCMR1_CC2S_Pos);

			//configuramos el filtro (sin filtro)
			ptrCaptureHandler->ptrTIMx->CCMR1 &= ~(0xF << TIM_CCMR1_IC2F_Pos);
			ptrCaptureHandler->ptrTIMx->CCMR1 |= (0x3 << TIM_CCMR1_IC2F_Pos);

			//configuramos el preescaler
			ptrCaptureHandler->ptrTIMx->CCMR1 &= ~(0x3 << TIM_CCMR1_IC2PSC_Pos);
			ptrCaptureHandler->ptrTIMx->CCMR1 |= (ptrCaptureHandler->config.prescalerCapture << TIM_CCMR1_IC2PSC_Pos);

			// Configuramos el flanco que deseamos capturar
			if (ptrCaptureHandler->config.edgeSignal == CAPTURE_RISING_EDGE){
				//Configuramos 00, para rising edge
				ptrCaptureHandler->ptrTIMx->CCER &= ~(TIM_CCER_CC2P);
				ptrCaptureHandler->ptrTIMx->CCER &= ~(TIM_CCER_CC2NP);
			}else{
				//Configuramos 01, para falling edge
				ptrCaptureHandler->ptrTIMx->CCER |= (TIM_CCER_CC2P);
				ptrCaptureHandler->ptrTIMx->CCER &= ~(TIM_CCER_CC2NP);
			}

			//Ademas activamos las interrupciones por conteo de tiempo en flanco
			ptrCaptureHandler->ptrTIMx->DIER |= TIM_DIER_CC2IE;
			break;
		case CAPTURE_CHANNEL_3:
			//borramos el posible valor cargado
			ptrCaptureHandler->ptrTIMx->CCMR2 &= ~(TIM_CCMR2_CC3S);
			//Configuramos como entrada en el mismo canal
			ptrCaptureHandler->ptrTIMx->CCMR2 = (1 << TIM_CCMR2_CC3S_Pos);

			//configuramos el filtro (sin filtro)
			ptrCaptureHandler->ptrTIMx->CCMR2 &= ~(0xF << TIM_CCMR2_IC3F_Pos);
			ptrCaptureHandler->ptrTIMx->CCMR2 |= (0x3 << TIM_CCMR2_IC3F_Pos);

			//configuramos el preescaler
			ptrCaptureHandler->ptrTIMx->CCMR2 &= ~(0x3 << TIM_CCMR2_IC3PSC_Pos);
			ptrCaptureHandler->ptrTIMx->CCMR2 |= (ptrCaptureHandler->config.prescalerCapture << TIM_CCMR2_IC3PSC_Pos);

			// Configuramos el flanco que deseamos capturar
			if (ptrCaptureHandler->config.edgeSignal == CAPTURE_RISING_EDGE){
				//Configuramos 00, para rising edge
				ptrCaptureHandler->ptrTIMx->CCER &= ~(TIM_CCER_CC3P);
				ptrCaptureHandler->ptrTIMx->CCER &= ~(TIM_CCER_CC3NP);
			}else{
				//Configuramos 01, para falling edge
				ptrCaptureHandler->ptrTIMx->CCER |= (TIM_CCER_CC3P);
				ptrCaptureHandler->ptrTIMx->CCER &= ~(TIM_CCER_CC3NP);
			}

			//Ademas activamos las interrupciones por conteo de tiempo en flanco
			ptrCaptureHandler->ptrTIMx->DIER |= TIM_DIER_CC3IE;

			break;
		case CAPTURE_CHANNEL_4:
			//borramos el posible valor cargado
			ptrCaptureHandler->ptrTIMx->CCMR2 &= ~(TIM_CCMR2_CC4S);
			//Configuramos como entrada en el mismo canal
			ptrCaptureHandler->ptrTIMx->CCMR2 = (1 << TIM_CCMR2_CC4S_Pos);

			//configuramos el filtro (sin filtro)
			ptrCaptureHandler->ptrTIMx->CCMR2 &= ~(0xF << TIM_CCMR2_IC4F_Pos);
			ptrCaptureHandler->ptrTIMx->CCMR2 |= (0x3 << TIM_CCMR2_IC4F_Pos);

			//configuramos el preescaler
			ptrCaptureHandler->ptrTIMx->CCMR2 &= ~(0x3 << TIM_CCMR2_IC4PSC_Pos);
			ptrCaptureHandler->ptrTIMx->CCMR2 |= (ptrCaptureHandler->config.prescalerCapture << TIM_CCMR2_IC4PSC_Pos);

			// Configuramos el flanco que deseamos capturar
			if (ptrCaptureHandler->config.edgeSignal == CAPTURE_RISING_EDGE){
				//Configuramos 00, para rising edge
				ptrCaptureHandler->ptrTIMx->CCER &= ~(TIM_CCER_CC4P);
				ptrCaptureHandler->ptrTIMx->CCER &= ~(TIM_CCER_CC4NP);
			}else{
				//Configuramos 01, para falling edge
				ptrCaptureHandler->ptrTIMx->CCER |= (TIM_CCER_CC4P);
				ptrCaptureHandler->ptrTIMx->CCER &= ~(TIM_CCER_CC4NP);
			}


			//Ademas activamos las interrupciones por conteo de tiempo en flanco
			ptrCaptureHandler->ptrTIMx->DIER |= TIM_DIER_CC4IE;

			break;
		default:
			__NOP();
			break;
	}//Fin del switch case

	if(ptrCaptureHandler->ptrTIMx == TIM2){
		// Activando en NVIC para la interrupción del TIM2
		NVIC_EnableIRQ(TIM2_IRQn);
	}
	else if(ptrCaptureHandler->ptrTIMx == TIM3){
		// Activando en NVIC para la interrupción del TIM3
		NVIC_EnableIRQ(TIM3_IRQn);
	}
	else if(ptrCaptureHandler->ptrTIMx == TIM4){
		// Activando en NVIC para la interrupción del TIM4
		NVIC_EnableIRQ(TIM4_IRQn);
	}
	else if(ptrCaptureHandler->ptrTIMx == TIM5){
		// Activando en NVIC para la interrupción del TIM5
		NVIC_EnableIRQ(TIM5_IRQn);
	}
	else{
		__NOP();
	}

	//Configuramos el preescaler del timer, el cual define a que velocidad se incrementa nuestro timer
	ptrCaptureHandler->ptrTIMx->PSC = ptrCaptureHandler->config.timerSpeed;

	__enable_irq();


}

void startCapture (Capture_Handler_t *ptrCaptureHandler){
	switch (ptrCaptureHandler->config.channel) {
			case CAPTURE_CHANNEL_1:
				//Activamos el modo captura
				ptrCaptureHandler->ptrTIMx->CCER |= TIM_CCER_CC1E;
				break;
			case CAPTURE_CHANNEL_2:
				//Activamos el modo captura
				ptrCaptureHandler->ptrTIMx->CCER |= TIM_CCER_CC2E;

				break;
			case CAPTURE_CHANNEL_3:
				//Activamos el modo captura
				ptrCaptureHandler->ptrTIMx->CCER |= TIM_CCER_CC3E;

				break;
			case CAPTURE_CHANNEL_4:
				//Activamos el modo captura
				ptrCaptureHandler->ptrTIMx->CCER |= TIM_CCER_CC4E;

				break;
			default:
				__NOP();
				break;
		}
		// Comenzamos el conteo del timer
		ptrCaptureHandler->ptrTIMx->CR1 |= TIM_CR1_CEN;
}

void stopCapture (Capture_Handler_t *ptrCaptureHandler){
	// Paramos el conteo del timer
	ptrCaptureHandler->ptrTIMx->CR1 &= ~TIM_CR1_CEN;
	switch (ptrCaptureHandler->config.channel) {
		case CAPTURE_CHANNEL_1:

			//Activamos el modo captura
			ptrCaptureHandler->ptrTIMx->CCER &= ~TIM_CCER_CC1E;


			break;
		case CAPTURE_CHANNEL_2:
			//Activamos el modo captura
			ptrCaptureHandler->ptrTIMx->CCER &= ~TIM_CCER_CC2E;

			break;
		case CAPTURE_CHANNEL_3:

			//Activamos el modo captura
			ptrCaptureHandler->ptrTIMx->CCER &= ~TIM_CCER_CC3E;

			break;
		case CAPTURE_CHANNEL_4:

			//Activamos el modo captura
			ptrCaptureHandler->ptrTIMx->CCER &= ~TIM_CCER_CC4E;

			break;
		default:
			__NOP();
			break;
	}

}



uint32_t timeStamp(Capture_Handler_t *ptrCaptureHandler){

	uint32_t timestamp = 0;

	switch(ptrCaptureHandler->config.channel){
		case CAPTURE_CHANNEL_1:{

			// Capturamos el valor del tiempo almacenado en el CCRx
			timestamp = ptrCaptureHandler->ptrTIMx->CCR1;

			break;
		}
		case CAPTURE_CHANNEL_2:{

			// Capturamos el valor del tiempo almacenado en el CCRx
			timestamp = ptrCaptureHandler->ptrTIMx->CCR2;


			break;
		}
		case CAPTURE_CHANNEL_3:{

			// Capturamos el valor del tiempo almacenado en el CCRx
			timestamp = ptrCaptureHandler->ptrTIMx->CCR3;
			break;
		}
		case CAPTURE_CHANNEL_4:{

			// Capturamos el valor del tiempo almacenado en el CCRx
			timestamp = ptrCaptureHandler->ptrTIMx->CCR4;

			break;
		}
		default: {
			break;
		}
	}

	return timestamp;

}


uint32_t getFreq(Capture_Handler_t *ptrCaptureHandler, uint32_t ts1, uint32_t ts2){


	int32_t delta = 0;
	switch (ptrCaptureHandler->config.prescalerCapture) {
		case CAPTURE_PREESCALER_1_1:
			delta = abs(ts2-ts1);
			break;
		case CAPTURE_PREESCALER_2_1:
			delta = abs(ts2-ts1)/2;
			break;
		case CAPTURE_PREESCALER_4_1:
			delta = abs(ts2-ts1)/4;
			break;
		case CAPTURE_PREESCALER_8_1:
			delta = abs(ts2-ts1)/8;
			break;
		default:
			__NOP();
			break;
	}

	delta = (delta*ptrCaptureHandler->config.timerSpeed)/16000;

	return delta;

}

void clean(Capture_Handler_t *ptrCaptureHandler){

	ptrCaptureHandler->ptrTIMx->CNT = 0;

	switch(ptrCaptureHandler->config.channel){
		case CAPTURE_CHANNEL_1:{
			ptrCaptureHandler->ptrTIMx->CCR1 = 0;
			break;
		}
		case CAPTURE_CHANNEL_2:{
			ptrCaptureHandler->ptrTIMx->CCR2 = 0;
			break;
		}
		case CAPTURE_CHANNEL_3:{
			ptrCaptureHandler->ptrTIMx->CCR3 = 0;
			break;
		}
		case CAPTURE_CHANNEL_4:{
			ptrCaptureHandler->ptrTIMx->CCR4 = 0;
			break;
		}
		default: {
			break;
		}
	}
}

