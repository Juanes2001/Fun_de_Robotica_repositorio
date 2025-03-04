/*
 * PwmDriver.c
 *
 *  Created on: XXXX , 2022
 *      Author: namontoy
 */
#include "PwmDriver.h"
#include "assert.h"
#include <math.h>

uint16_t periodo = 0;


/**/
void pwm_Config(PWM_Handler_t *ptrPwmHandler){

	/* 1. Activar la señal de reloj del periférico requerido */
	if (ptrPwmHandler->ptrTIMx == TIM1){
		RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	}else if(ptrPwmHandler->ptrTIMx == TIM2){
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	}
	else if(ptrPwmHandler->ptrTIMx == TIM3){
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	}
	else if(ptrPwmHandler->ptrTIMx == TIM4){
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	}
	else if(ptrPwmHandler->ptrTIMx == TIM5){
		RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	}
	else{
		__NOP();
	}

	//Dejamos una relacion 1 a 1 para la velocidad de conteo del timer
	ptrPwmHandler->ptrTIMx->CR1 &= ~(TIM_CR1_CKD);




	if (ptrPwmHandler->config.one_pulse == PWM_DISABLE_ONE_PULSE){

		// preguntamos si se estan usando optoacopladores

		if(ptrPwmHandler->config.optocoupler == PWM_DISABLE_OPTOCOUPLER){
			/* 1. Cargamos la frecuencia deseada */
			setFrequency(ptrPwmHandler);

			/* 2. Cargamos el valor del dutty-Cycle*/
			setDuttyCycle(ptrPwmHandler);
		}else{

			/* 1. Cargamos la frecuencia deseada */
			setFrequency(ptrPwmHandler);

			/* 2. Cargamos el valor del dutty-Cycle*/
			setDuttyCycleAfOpt(ptrPwmHandler);
		}

		/* 2a. Estamos en UP_Mode, el limite se carga en ARR y se comienza en 0 */
		ptrPwmHandler->ptrTIMx->CR1 &= ~TIM_CR1_DIR;


		ptrPwmHandler->ptrTIMx->CNT = 0;

		/* 3. Configuramos los bits CCxS del registro TIMy_CCMR1, de forma que sea modo salida
		 * (para cada canal hay un conjunto CCxS)
		 *
		 * 4. Además, en el mismo "case" podemos configurar el modo del PWM, su polaridad...
		 *
		 * 5. Y además activamos el preload bit, para que cada vez que exista un update-event
		 * el valor cargado en el CCRx será recargado en el registro "shadow" del PWM */
		outputSelector(ptrPwmHandler);

	}else{

		// Si llegamos aca es porque se desea configurar el modo un pulso

		// Activamos el modo de un pulso
		ptrPwmHandler->ptrTIMx->CR1 |= TIM_CR1_OPM;


		// Como queremos accionar el pulso de luz mediante un trigger externo por los canales de input
		// Configuramos el canal correspondiente a el input en el que entrará el trigger externo

		switch(ptrPwmHandler->config.channel_in){
			case PWM_IN_CHANNEL_1:{

				// Seleccionamos como entrada el canal

				ptrPwmHandler->ptrTIMx->CCMR1 &= ~TIM_CCMR1_CC1S;

				ptrPwmHandler->ptrTIMx->CCMR1 |= ((0*TIM_CCMR1_CC1S_1) | TIM_CCMR1_CC1S_0);

				// Configuramos que queremos una deteccion de subida
				ptrPwmHandler->ptrTIMx->CCER &= ~TIM_CCER_CC1P;
				ptrPwmHandler->ptrTIMx->CCER &= ~TIM_CCER_CC1NP;


				// En modo slave le diremos configuramos que el trigger input seleccionado es
				// esta en modo trigger.

				ptrPwmHandler->ptrTIMx->SMCR &= ~TIM_SMCR_TS;

				ptrPwmHandler->ptrTIMx->SMCR |= (TIM_SMCR_TS_2) | ((0*TIM_SMCR_TS_1) | TIM_SMCR_TS_0);

				// Configuramos el Slave mode a modo de trigger mode de tal forma que el counter inicie
				// cuando llegue señal de entrada (flanco de subida o bajada), solo inicia el counter mas no lo resetea
				ptrPwmHandler->ptrTIMx->SMCR &= ~TIM_SMCR_SMS;

				ptrPwmHandler->ptrTIMx->SMCR |= TIM_SMCR_SMS_2 | (TIM_SMCR_SMS_1 | (0*TIM_SMCR_SMS_0));

				// Configuramos el canal como PWM modo 2
				ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | (0*TIM_CCMR1_OC1M_0);

				// Activamos la funcionalidad de pre-load
				ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC1PE;

				break;
			}

			case PWM_IN_CHANNEL_2:{

				// Seleccionamos como entrada el canal

				ptrPwmHandler->ptrTIMx->CCMR1 &= ~TIM_CCMR1_CC2S;

				ptrPwmHandler->ptrTIMx->CCMR1 |= ((0*TIM_CCMR1_CC2S_1) | TIM_CCMR1_CC2S_0);

				// Configuramos que queremos una deteccion de subida
				ptrPwmHandler->ptrTIMx->CCER &= ~TIM_CCER_CC2P;
				ptrPwmHandler->ptrTIMx->CCER &= ~TIM_CCER_CC2NP;


				// En modo slave le diremos configuramos que el trigger input seleccionado es
				// esta en modo trigger.

				ptrPwmHandler->ptrTIMx->SMCR &= ~TIM_SMCR_TS;

				ptrPwmHandler->ptrTIMx->SMCR |= (TIM_SMCR_TS_2) | (TIM_SMCR_TS_1 | (0*TIM_SMCR_TS_0));

				// Configuramos el Slave mode a modo de trigger mode de tal forma que el counter inicie
				// cuando llegue señal de entrada (flanco de subida o bajada), solo inicia el counter mas no lo resetea
				ptrPwmHandler->ptrTIMx->SMCR &= ~TIM_SMCR_SMS;

				ptrPwmHandler->ptrTIMx->SMCR |= TIM_SMCR_SMS_2 | (TIM_SMCR_SMS_1 | (0*TIM_SMCR_SMS_0));

				// Configuramos el canal como PWM modo 2
				ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | (0*TIM_CCMR1_OC2M_0);

				// Activamos la funcionalidad de pre-load
				ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC2PE;

				break;
			}


			default:{
				break;
			}

		}// fin del switch-case

		switch(ptrPwmHandler->config.channel){
			case PWM_CHANNEL_1:{

				// Seleccionamos como salida el canal
				ptrPwmHandler->ptrTIMx->CCMR1 &= ~TIM_CCMR1_CC1S;

				// Configuramos el canal como PWM modo 1
				ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;

				// Activamos la funcionalidad de pre-load
				ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC1PE;

				break;
			}

			case PWM_CHANNEL_2:{

				// Seleccionamos como salida el canal
				ptrPwmHandler->ptrTIMx->CCMR1 &= ~TIM_CCMR1_CC2S;

				// Configuramos el canal como PWM modo 2
				ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;

				// Activamos la funcionalidad de pre-load
				ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC2PE;

				break;
			}


			default:{
				break;
			}

		}// fin del switch-case


		// preguntamos si se estan usando optoacopladores

			if(ptrPwmHandler->config.optocoupler == PWM_DISABLE_OPTOCOUPLER){
				/* 1. Cargamos la frecuencia deseada */
				setFrequency(ptrPwmHandler);

				/* 2. Cargamos el valor del dutty-Cycle*/
				setDuttyCycle(ptrPwmHandler);
			}else{

				/* 1. Cargamos la frecuencia deseada */
				setFrequency(ptrPwmHandler);

				/* 2. Cargamos el valor del dutty-Cycle*/
				setDuttyCycleAfOpt(ptrPwmHandler);
			}

			/* 2a. Estamos en UP_Mode, el limite se carga en ARR y se comienza en 0 */
			ptrPwmHandler->ptrTIMx->CR1 &= ~TIM_CR1_DIR;


			ptrPwmHandler->ptrTIMx->CNT = 0;



	}// fin del if else

}// fin de la funcion


// Selector de output y tipo de PWM
void outputSelector(PWM_Handler_t *ptrPwmHandler){

	switch(ptrPwmHandler->config.channel){
		case PWM_CHANNEL_1:{
			// Seleccionamos como salida el canal
			ptrPwmHandler->ptrTIMx->CCMR1 &= ~TIM_CCMR1_CC1S;

			// Configuramos el canal como PWM
			ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

			// Activamos la funcionalidad de pre-load
			ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC1PE;


			break;
		}

		case PWM_CHANNEL_2:{
			// Seleccionamos como salida el canal
			ptrPwmHandler->ptrTIMx->CCMR1 &= ~TIM_CCMR1_CC2S;

			// Configuramos el canal como PWM
			ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;

			// Activamos la funcionalidad de pre-load
			ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC2PE;

			break;
		}

		case PWM_CHANNEL_3:{
				// Seleccionamos como salida el canal
			ptrPwmHandler->ptrTIMx->CCMR2 &= ~TIM_CCMR2_CC3S;

			// Configuramos el canal como PWM
			ptrPwmHandler->ptrTIMx->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;

			// Activamos la funcionalidad de pre-load
			ptrPwmHandler->ptrTIMx->CCMR2 |= TIM_CCMR2_OC3PE;

			break;
		}
		case PWM_CHANNEL_4:{
				// Seleccionamos como salida el canal
			ptrPwmHandler->ptrTIMx->CCMR2 &= ~TIM_CCMR2_CC4S;

			// Configuramos el canal como PWM
			ptrPwmHandler->ptrTIMx->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;

			// Activamos la funcionalidad de pre-load
			ptrPwmHandler->ptrTIMx->CCMR2 |= TIM_CCMR2_OC4PE;

			break;
		}

		default:{
			break;
		}

	}// fin del switch-case

}


/* Función para activar el Timer y activar todo el módulo PWM */
void startPwmSignal(PWM_Handler_t *ptrPwmHandler) {
	ptrPwmHandler->ptrTIMx->CR1 |= TIM_CR1_CEN;

}

/* Función para desactivar el Timer y detener todo el módulo PWM*/
void stopPwmSignal(PWM_Handler_t *ptrPwmHandler) {
	ptrPwmHandler->ptrTIMx->CR1 &= ~TIM_CR1_CEN;
}

/* Función encargada de activar cada uno de los canales con los que cuenta el TimerX */
uint8_t enableOutput(PWM_Handler_t *ptrPwmHandler) {

	if (ptrPwmHandler->ptrTIMx == TIM1){
		// Para el caso de Timer 1, debemos primero activar la opcion MOE en el BDTR register

		ptrPwmHandler->ptrTIMx->BDTR |= TIM_BDTR_MOE;

		switch (ptrPwmHandler->config.channel) {
			case PWM_CHANNEL_1: {
				// Activamos la salida del canal 1
				ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC1E;

				break;
			}

			case PWM_CHANNEL_2: {
				// Activamos la salida del canal 2
				ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC2E;

				break;
			}

			case PWM_CHANNEL_3: {
				// Activamos la salida del canal 3
				ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC3E;

				break;
			}

			case PWM_CHANNEL_4: {
				// Activamos la salida del canal 4
				ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC4E;

				break;
			}

			default: {
				break;
			}

		}

	}else{
		switch (ptrPwmHandler->config.channel) {
			case PWM_CHANNEL_1: {
				// Activamos la salida del canal 1
				ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC1E;

				break;
			}

			case PWM_CHANNEL_2: {
				// Activamos la salida del canal 2
				ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC2E;

				break;
			}

			case PWM_CHANNEL_3: {
				// Activamos la salida del canal 3
				ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC3E;

				break;
			}

			case PWM_CHANNEL_4: {
				// Activamos la salida del canal 4
				ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC4E;

				break;
			}

			default: {
				break;
			}

		}
	}
	return SET;
}




/* Función encargada de activar cada uno de los canales con los que cuenta el TimerX */
uint8_t enableComplementaryOutput(PWM_Handler_t *ptrPwmHandler) {

	if (ptrPwmHandler->ptrTIMx == TIM1){
		// Para el caso de Timer 1, devemos primero activar la opcion MOE en el BDTR register

		ptrPwmHandler->ptrTIMx->BDTR |= TIM_BDTR_MOE;

		switch (ptrPwmHandler->config.channel) {
			case PWM_CHANNEL_1: {
				// Activamos la salida del canal 1
				ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC1NE;

				break;
			}

			case PWM_CHANNEL_2: {
				// Activamos la salida del canal 2
				ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC2NE;

				break;
			}

			case PWM_CHANNEL_3: {
				// Activamos la salida del canal 3
				ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC3NE;

				break;
			}


			default: {
				break;
			}

		}

	}else{
		switch (ptrPwmHandler->config.channel) {
			case PWM_CHANNEL_1: {
				// Activamos la salida del canal 1
				ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC1NE;

				break;
			}

			case PWM_CHANNEL_2: {
				// Activamos la salida del canal 2
				ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC2NE;

				break;
			}

			case PWM_CHANNEL_3: {
				// Activamos la salida del canal 3
				ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC3NE;

				break;
			}// NO EXISTE EL CANAL COMPLEMENTARIO EN EL CANAL 4

			default: {
				break;
			}

		}
	}
	return SET;
}

/* 
 * La frecuencia es definida por el conjunto formado por el preescaler (PSC)
 * y el valor límite al que llega el Timer (ARR), con estos dos se establece
 * la frecuencia.
 * */
void setFrequency(PWM_Handler_t *ptrPwmHandler){

	uint32_t speed   = 0;

	// Cargamos el valor del prescaler, nos define la velocidad (en ns) a la cual
	// se incrementa el Timer
	ptrPwmHandler->ptrTIMx->PSC = ptrPwmHandler->config.prescaler;

	speed = ptrPwmHandler->config.prescaler;

	// Cargamos el valor del ARR, el cual es el límite de incrementos del Timer
	// antes de hacer un update y reload.


	if((speed == PWM_SPEED_16MHz_1us )
     ||(speed == PWM_SPEED_20MHz_1us)
     ||(speed == PWM_SPEED_30MHz_1us)
     ||(speed == PWM_SPEED_40MHz_1us)
     ||(speed == PWM_SPEED_50MHz_1us)
     ||(speed == PWM_SPEED_60MHz_1us)
     ||(speed == PWM_SPEED_70MHz_1us)
     ||(speed == PWM_SPEED_80MHz_1us)
     ||(speed == PWM_SPEED_90MHz_1us)
     ||(speed == PWM_SPEED_100MHz_1us)){

		periodo = ptrPwmHandler->config.periodo * 1000 ;

		ptrPwmHandler->ptrTIMx->ARR = periodo - 1;


	}else if ((speed == PWM_SPEED_16MHz_10us )
	  ||(speed == PWM_SPEED_20MHz_10us)
	  ||(speed == PWM_SPEED_30MHz_10us)
	  ||(speed == PWM_SPEED_40MHz_10us)
	  ||(speed == PWM_SPEED_50MHz_10us)
	  ||(speed == PWM_SPEED_60MHz_10us)
	  ||(speed == PWM_SPEED_70MHz_10us)
	  ||(speed == PWM_SPEED_80MHz_10us)
	  ||(speed == PWM_SPEED_90MHz_10us)
	  ||(speed == PWM_SPEED_100MHz_10us)){

		periodo = ptrPwmHandler->config.periodo * 100 ;

		ptrPwmHandler->ptrTIMx->ARR = periodo - 1;

	}else if ((speed == PWM_SPEED_16MHz_100us )
		   || (speed == PWM_SPEED_20MHz_100us)
		   || (speed == PWM_SPEED_30MHz_100us)
		   || (speed == PWM_SPEED_40MHz_100us)
		   || (speed == PWM_SPEED_50MHz_100us)
		   || (speed == PWM_SPEED_60MHz_100us)
		   || (speed == PWM_SPEED_70MHz_100us)
		   || (speed == PWM_SPEED_80MHz_100us)
		   || (speed == PWM_SPEED_90MHz_100us)
		   || (speed == PWM_SPEED_100MHz_100us)){

		periodo = ptrPwmHandler->config.periodo * 10   ;

		ptrPwmHandler->ptrTIMx->ARR = periodo - 1;



	}else if ((speed == PWM_SPEED_16MHz_1ms)
		   || (speed == PWM_SPEED_20MHz_1ms)
		   || (speed == PWM_SPEED_30MHz_1ms)
		   || (speed == PWM_SPEED_40MHz_1ms)
		   || (speed == PWM_SPEED_50MHz_1ms)
		   || (speed == PWM_SPEED_60MHz_1ms)
		   || (speed == PWM_SPEED_70MHz_1ms)
		   || (speed == PWM_SPEED_80MHz_1ms)
		   || (speed == PWM_SPEED_90MHz_1ms)
		   || (speed == PWM_SPEED_100MHz_1ms)){

		periodo = ptrPwmHandler->config.periodo;

		ptrPwmHandler->ptrTIMx->ARR = periodo - 1;

	}else{
		periodo = ptrPwmHandler->config.periodo / 20;  //Se tiene el caso mas minimo posible, donde el contador cuenta cada 10 nanosegundos

		ptrPwmHandler->ptrTIMx->ARR = periodo - 1;
	}

}


/* Función para actualizar la frecuencia, funciona de la mano con setFrequency */
void updateFrequency(PWM_Handler_t *ptrPwmHandler, uint16_t freq){
	// Actualizamos el registro que manipula el periodo

	ptrPwmHandler->config.periodo = (uint16_t) 1/freq;


	// Llamamos a la fucnión que cambia la frecuencia
	setFrequency(ptrPwmHandler);
}

/* El valor del dutty debe estar dado en valores de %, entre 0% y 100%*/
void setDuttyCycle(PWM_Handler_t *ptrPwmHandler){

	// Seleccionamos el canal para configurar su dutty
	switch(ptrPwmHandler->config.channel){
	case PWM_CHANNEL_1:{
		double op = (ptrPwmHandler->config.duttyCicle) * periodo;
		ptrPwmHandler->ptrTIMx->CCR1 = (op)/100 -1;
		break;
	}

	case PWM_CHANNEL_2:{
		double op = (ptrPwmHandler->config.duttyCicle) * periodo;
		ptrPwmHandler->ptrTIMx->CCR2 = (op)/100 -1;
		break;
	}

	case PWM_CHANNEL_3:{
		double op = (ptrPwmHandler->config.duttyCicle) * periodo;
		ptrPwmHandler->ptrTIMx->CCR3 = (op)/100 -1;
		break;
	}

	case PWM_CHANNEL_4:{
		double op = (ptrPwmHandler->config.duttyCicle) * periodo;
		ptrPwmHandler->ptrTIMx->CCR4 = (op)/100 -1;
		break;
	}

	default:{
		break;
	}

	}// fin del switch-case

}


void setDuttyCycleAfOpt(PWM_Handler_t *ptrPwmHandler){

	// Seleccionamos el canal para configurar su dutty
	switch(ptrPwmHandler->config.channel){
	case PWM_CHANNEL_1:{
		double op = (100-ptrPwmHandler->config.duttyCicle) * periodo;
		ptrPwmHandler->ptrTIMx->CCR1 = (op)/100 - 1 ;
		break;
	}

	case PWM_CHANNEL_2:{
		double op = (100-ptrPwmHandler->config.duttyCicle) * periodo;
		ptrPwmHandler->ptrTIMx->CCR2 = (op)/100 - 1;
		break;
	}

	case PWM_CHANNEL_3:{
		double op = (100-ptrPwmHandler->config.duttyCicle) * periodo;
		ptrPwmHandler->ptrTIMx->CCR3 = (op)/100 - 1;
		break;
	}

	case PWM_CHANNEL_4:{
		double op = (100-ptrPwmHandler->config.duttyCicle) * periodo;
		ptrPwmHandler->ptrTIMx->CCR4 = (op)/100 - 1;
		break;
	}

	default:{
		break;
	}

	}// fin del switch-case


}

uint8_t showPWM (PWM_Handler_t *ptrPwmHandler){

	uint8_t PWMdutty = 0;

	// Seleccionamos el canal para configurar su dutty
	switch(ptrPwmHandler->config.channel){
	case PWM_CHANNEL_1:{

		PWMdutty = (ptrPwmHandler->ptrTIMx->CCR1)*100 / periodo;
		break;
	}

	case PWM_CHANNEL_2:{
		PWMdutty = (ptrPwmHandler->ptrTIMx->CCR2)*100 / periodo;
		break;
	}

	case PWM_CHANNEL_3:{
		PWMdutty = (ptrPwmHandler->ptrTIMx->CCR3)*100 / periodo;
		break;
	}

	case PWM_CHANNEL_4:{
		PWMdutty = (ptrPwmHandler->ptrTIMx->CCR4)*100 / periodo;
		break;
	}

	default:{
		break;
	}

	}// fin del switch-case

	return PWMdutty;

}

uint8_t showPWMBfOpt (PWM_Handler_t *ptrPwmHandler){

	uint8_t PWMdutty = 0;

	// Seleccionamos el canal para configurar su dutty
	switch(ptrPwmHandler->config.channel){
	case PWM_CHANNEL_1:{

		PWMdutty = 100-(ptrPwmHandler->ptrTIMx->CCR1)*100 / periodo;
		break;
	}

	case PWM_CHANNEL_2:{
		PWMdutty = 100-(ptrPwmHandler->ptrTIMx->CCR2)*100 / periodo;
		break;
	}

	case PWM_CHANNEL_3:{
		PWMdutty = 100-(ptrPwmHandler->ptrTIMx->CCR3)*100 / periodo;
		break;
	}

	case PWM_CHANNEL_4:{
		PWMdutty = 100-(ptrPwmHandler->ptrTIMx->CCR4)*100 / periodo;
		break;
	}

	default:{
		break;
	}

	}// fin del switch-case

	return PWMdutty;

}


/* Función para actualizar el Dutty, funciona de la mano con setDuttyCycle */
void updateDuttyCycle(PWM_Handler_t *ptrPwmHandler, float newDutty){
	// Actualizamos el registro que manipula el dutty
    ptrPwmHandler->config.duttyCicle = newDutty;

	// Llamamos a la fucnión que cambia el dutty y cargamos el nuevo valor
    setDuttyCycle(ptrPwmHandler);
}


void updateDuttyCycleAfOpt(PWM_Handler_t *ptrPwmHandler, float newDutty){

	// Actualizamos el registro que manipula el dutty
	ptrPwmHandler->config.duttyCicle = newDutty;

	// Llamamos a la fucnión que cambia el dutty y cargamos el nuevo valor
	setDuttyCycleAfOpt(ptrPwmHandler);

}

void enableEvent(PWM_Handler_t *ptrPwmHandler){
	switch (ptrPwmHandler->config.channel){
	case PWM_CHANNEL_1: {
		ptrPwmHandler->ptrTIMx->EGR |= TIM_EGR_CC1G;
		break;
	}
	case PWM_CHANNEL_2: {
		ptrPwmHandler->ptrTIMx->EGR |= TIM_EGR_CC2G;
		break;
	}
	case PWM_CHANNEL_3: {
		ptrPwmHandler->ptrTIMx->EGR |= TIM_EGR_CC3G;
		break;
	}
	case PWM_CHANNEL_4: {
		ptrPwmHandler->ptrTIMx->EGR |= TIM_EGR_CC4G;
		break;
	}
	default:{
		break;
	}
	}
}

void disableEvent(PWM_Handler_t *ptrPwmHandler){
	switch (ptrPwmHandler->config.channel){
		case PWM_CHANNEL_1: {
			ptrPwmHandler->ptrTIMx->EGR &= ~TIM_EGR_CC1G;
			break;
		}
		case PWM_CHANNEL_2: {
			ptrPwmHandler->ptrTIMx->EGR &= ~TIM_EGR_CC2G;
			break;
		}
		case PWM_CHANNEL_3: {
			ptrPwmHandler->ptrTIMx->EGR &= ~TIM_EGR_CC3G;
			break;
		}
		case PWM_CHANNEL_4: {
			ptrPwmHandler->ptrTIMx->EGR &= ~TIM_EGR_CC4G;
			break;
		}
		default:{
			break;
		}
		}
}


uint8_t disableOutput(PWM_Handler_t *ptrPwmHandler){

	switch (ptrPwmHandler->config.channel) {
		case PWM_CHANNEL_1: {
			// Activamos la salida del canal 1
			ptrPwmHandler->ptrTIMx->CCER &= ~TIM_CCER_CC1E;

			break;
		}

		case PWM_CHANNEL_2: {
			// Activamos la salida del canal 2
			ptrPwmHandler->ptrTIMx->CCER &= ~TIM_CCER_CC2E;

			break;
		}

		case PWM_CHANNEL_3: {
			// Activamos la salida del canal 3
			ptrPwmHandler->ptrTIMx->CCER &= ~TIM_CCER_CC3E;

			break;
		}

		case PWM_CHANNEL_4: {
			// Activamos la salida del canal 4
			ptrPwmHandler->ptrTIMx->CCER &= ~TIM_CCER_CC4E;

			break;
		}

		default: {
			break;
		}
		}

	return RESET;
}

void PWMx_Toggle(PWM_Handler_t *ptrPwmHandler){

	switch (ptrPwmHandler->config.channel) {
			case PWM_CHANNEL_1: {
				// Activamos el polarity en este canal
				ptrPwmHandler->ptrTIMx->CCER ^= TIM_CCER_CC1P;

				break;
			}

			case PWM_CHANNEL_2: {
				// Activamos el polarity en este canal
				ptrPwmHandler->ptrTIMx->CCER ^= TIM_CCER_CC2P;

				break;
			}

			case PWM_CHANNEL_3: {
				// Activamos el polarity en este canal
				ptrPwmHandler->ptrTIMx->CCER ^= TIM_CCER_CC3P;

				break;
			}

			case PWM_CHANNEL_4: {
				// Activamos el polarity en este canal
				ptrPwmHandler->ptrTIMx->CCER ^= TIM_CCER_CC4P;

				break;
			}

			default: {
				break;
			}
			}


}



