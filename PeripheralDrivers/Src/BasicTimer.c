/*
 * BasicTimer.c
 *
 *  Created on: Apr 18, 2022
 *      Author: Juan Esteban Rodriguez Ochoa
 */

#include "BasicTimer.h"

/* Función en la que cargamos la configuración del Timer
 * Recordar que siempre se debe comenzar con activar la señal de reloj
 * del periférico que se está utilizando.
 * Además, en este caso, debemos ser cuidadosos al momento de utilizar las interrupciones.
 * Los Timer están conectados directamente al elemento NVIC del Cortex-Mx
 * Debemos configurar y/o utilizar:
 *  - TIMx_CR1  (control Register 1)
 *  - TIMx_SMCR ( slave mode control register) -> mantener en 0 para modo Timer Básico
 *  - TIMx_DIER (DMA and Interrupt enable register)
 *  - TIMx_SR (Status register)
 *  - TIMx_CNT (Counter)
 *  - TIMx_PSC (Pre-scaler)
 *  - TIMx_ARR  (Auto-reload register)
 *
 *  Como vamos a trabajar con interrupciones, antes de configurar una nueva, debemos desactivar
 *  el sistema global de interrupciones, activar la IRQ específica y luego volver a encender
 *  el sistema.
 */


void inTIM4(void){

	//////////////////////////////////////////////////// /////////////////// //////////////////////////////////////////////

	////////////////////////////////Timer 4 para contador de tiempo ////////////////////////////////////

	handlerTIM4_time.ptrTIMx                           = TIM4;
	handlerTIM4_time.TIMx_Config.TIMx_interruptEnable  = BTIMER_DISABLE_INTERRUPT;
	handlerTIM4_time.TIMx_Config.TIMx_mode             = BTIMER_MODE_UP;
	handlerTIM4_time.TIMx_Config.TIMx_speed            = BTIMER_SPEED_100MHz_100us;
	handlerTIM4_time.TIMx_Config.TIMx_period           = 10;
	BasicTimer_Config(&handlerTIM4_time);

}

void BasicTimer_Config(BasicTimer_Handler_t *ptrBTimerHandler){

	uint32_t period = 0;
	uint32_t speed   = 0;

	/* 0. Desactivamos las interrupciones globales mientras configuramos el sistema.*/
	__disable_irq();

	/* 1. Activar la señal de reloj del periférico requerido */
	if (ptrBTimerHandler->ptrTIMx == TIM1){
		RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	}else if(ptrBTimerHandler->ptrTIMx == TIM2){
		// Registro del RCC que nos activa la señal de reloj para el TIM2
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	}
	else if(ptrBTimerHandler->ptrTIMx == TIM3){
		// Registro del RCC que nos activa la señal de reloj para el TIM3
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	}
	else if(ptrBTimerHandler->ptrTIMx == TIM4){
		// Registro del RCC que nos activa la señal de reloj para el TIM4
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	}
	else if(ptrBTimerHandler->ptrTIMx == TIM5){
		// Registro del RCC que nos activa la señal de reloj para el TIM5
		RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	}
	else{
		__NOP();
	}

	//Dejamos una relacion 1 a 1 para la velocidad de conteo del timer
	ptrBTimerHandler->ptrTIMx->CR1 &= ~(TIM_CR1_CKD);
	/* 2. Configuramos el Pre-scaler
	 * Recordar que el prescaler nos indica la velocidad a la que se incrementa el counter, de forma que
	 * periodo_incremento * veces_incremento_counter = periodo_update
	 * Modificar el valor del registro PSC en el TIM utilizado
	 */

	ptrBTimerHandler->ptrTIMx->PSC = ptrBTimerHandler->TIMx_Config.TIMx_speed;


	/* 3. Configuramos la dirección del counter (up/down)*/
	if(ptrBTimerHandler->TIMx_Config.TIMx_mode == BTIMER_MODE_UP){

		/* 3a. Estamos en UP_Mode, el limite se carga en ARR y se comienza en 0 */
		// Configurar el registro que nos controla el modo up or down
		ptrBTimerHandler->ptrTIMx->CR1 &= ~TIM_CR1_DIR;

		speed = ptrBTimerHandler->TIMx_Config.TIMx_speed;

		/* 3b. Configuramos el Auto-reload. Este es el "limite" hasta donde el CNT va a contar */
		if ((speed == BTIMER_SPEED_16MHz_10us )
		  ||(speed == BTIMER_SPEED_20MHz_10us)
		  ||(speed == BTIMER_SPEED_30MHz_10us)
		  ||(speed == BTIMER_SPEED_40MHz_10us)
		  ||(speed == BTIMER_SPEED_50MHz_10us)
		  ||(speed == BTIMER_SPEED_60MHz_10us)
		  ||(speed == BTIMER_SPEED_70MHz_10us)
		  ||(speed == BTIMER_SPEED_80MHz_10us)
		  ||(speed == BTIMER_SPEED_90MHz_10us)
		  ||(speed == BTIMER_SPEED_100MHz_10us)){

			period = ptrBTimerHandler->TIMx_Config.TIMx_period * 100 ;

			ptrBTimerHandler->ptrTIMx->ARR = period - 1;

		}else if ((speed == BTIMER_SPEED_16MHz_100us )
			   || (speed == BTIMER_SPEED_20MHz_100us)
			   || (speed == BTIMER_SPEED_30MHz_100us)
			   || (speed == BTIMER_SPEED_40MHz_100us)
			   || (speed == BTIMER_SPEED_50MHz_100us)
			   || (speed == BTIMER_SPEED_60MHz_100us)
			   || (speed == BTIMER_SPEED_70MHz_100us)
			   || (speed == BTIMER_SPEED_80MHz_100us)
			   || (speed == BTIMER_SPEED_90MHz_100us)
			   || (speed == BTIMER_SPEED_100MHz_100us)){

			period = ptrBTimerHandler->TIMx_Config.TIMx_period * 10   ;

			ptrBTimerHandler->ptrTIMx->ARR = period - 1;



		}else if ((speed == BTIMER_SPEED_16MHz_1ms)
			   || (speed == BTIMER_SPEED_20MHz_1ms)
			   || (speed == BTIMER_SPEED_30MHz_1ms)
			   || (speed == BTIMER_SPEED_40MHz_1ms)
			   || (speed == BTIMER_SPEED_50MHz_1ms)
			   || (speed == BTIMER_SPEED_60MHz_1ms)
			   || (speed == BTIMER_SPEED_70MHz_1ms)
			   || (speed == BTIMER_SPEED_80MHz_1ms)
			   || (speed == BTIMER_SPEED_90MHz_1ms)
			   || (speed == BTIMER_SPEED_100MHz_1ms)){

			period = ptrBTimerHandler->TIMx_Config.TIMx_period;

			ptrBTimerHandler->ptrTIMx->ARR = period - 1;

		}else{
			period = ptrBTimerHandler->TIMx_Config.TIMx_period / 10;  //Se tiene el caso mas minimo posible, donde el contador cuenta cada 10 nanosegundos

			ptrBTimerHandler->ptrTIMx->ARR = period - 1;
		}


		/* 3c. Reiniciamos el registro counter*/
		ptrBTimerHandler->ptrTIMx->CNT = 0;

	}else{
		/* 3a. Estamos en DOWN_Mode, el limite se carga en ARR (0) y se comienza en un valor alto
		 * Trabaja contando en direccion descendente*/
		/* Escriba codigo aca */
		ptrBTimerHandler->ptrTIMx->CR1 |= TIM_CR1_DIR;

		/* 3b. Configuramos el Auto-reload. Este es el "limite" hasta donde el CNT va a contar
		 * En modo descendente, con numero positivos, cual es el minimi valor al que ARR puede llegar*/
		/* Escriba codigo aca */
		ptrBTimerHandler->ptrTIMx->ARR = ptrBTimerHandler->TIMx_Config.TIMx_period - 1;

		/* 3c. Reiniciamos el registro counter
		 * Este es el valor con el que el counter comienza */
		ptrBTimerHandler->ptrTIMx->CNT = ptrBTimerHandler->TIMx_Config.TIMx_period - 1;
	}

	/* 4. Activamos el Timer (el CNT debe comenzar a contar*/
	//ptrBTimerHandler->ptrTIMx->CR1 |= TIM_CR1_CEN;

	/* 5. Activamos la interrupción debida al Timerx Utilizado
	 * Modificar el registro encargado de activar la interrupcion generada por el TIMx*/
	if (ptrBTimerHandler->TIMx_Config.TIMx_interruptEnable == BTIMER_ENABLE_INTERRUPT){

		ptrBTimerHandler->ptrTIMx->DIER |= TIM_DIER_UIE;

		/* 6. Activamos el canal del sistema NVIC para que lea la interrupción*/

		if(ptrBTimerHandler->ptrTIMx == TIM2){
			// Activando en NVIC para la interrupción del TIM2
			NVIC_EnableIRQ(TIM2_IRQn);
		}
		else if(ptrBTimerHandler->ptrTIMx == TIM3){
			// Activando en NVIC para la interrupción del TIM3
			NVIC_EnableIRQ(TIM3_IRQn);
		}
		else if(ptrBTimerHandler->ptrTIMx == TIM4){
			// Activando en NVIC para la interrupción del TIM4
			NVIC_EnableIRQ(TIM4_IRQn);
		}
		else if(ptrBTimerHandler->ptrTIMx == TIM5){
			// Activando en NVIC para la interrupción del TIM5
			NVIC_EnableIRQ(TIM5_IRQn);
		}
		else{
			__NOP();
		}


	}else{
		ptrBTimerHandler->ptrTIMx->DIER &= ~TIM_DIER_UIE;
	}


	/* 7. Volvemos a activar las interrupciones del sistema */
	__enable_irq();
}

void delay_ms(uint16_t time_to_wait_ms){

	startTimer(&handlerTIM4_time);
	// definimos una variable que almacenara el valor del counter en el timer 4
	uint16_t limit = (time_to_wait_ms * 10) - 1 ;
	uint16_t CNT   = 0;

	// comparamos el counter con el limit, y comenzamos a que cuente cada que el timer 4 haga una cuenta nueva
	while (CNT < limit){
		if (handlerTIM4_time.ptrTIMx->SR & TIM_SR_UIF)  {
			CNT += handlerTIM4_time.ptrTIMx->ARR + 1;
			handlerTIM4_time.ptrTIMx->SR &= ~TIM_SR_UIF;
		}
	}
	stopTimer(&handlerTIM4_time);
}


void TIM_SetPriority (BasicTimer_Handler_t *ptrBTimerHandler, uint8_t newPriority){

	__disable_irq();

	if(ptrBTimerHandler->ptrTIMx == TIM2){
		// Activando en NVIC para la interrupción del TIM2
		NVIC_SetPriority(TIM2_IRQn, newPriority);
	}
	else if(ptrBTimerHandler->ptrTIMx == TIM3){
		// Activando en NVIC para la interrupción del TIM3
		NVIC_SetPriority(TIM3_IRQn, newPriority);
	}
	else if(ptrBTimerHandler->ptrTIMx == TIM4){
		// Activando en NVIC para la interrupción del TIM4
		NVIC_SetPriority(TIM4_IRQn, newPriority);
	}
	else if(ptrBTimerHandler->ptrTIMx == TIM5){
		// Activando en NVIC para la interrupción del TIM5
		NVIC_SetPriority(TIM5_IRQn, newPriority);
	}
	else{
		__NOP();
	}

	__enable_irq();

}

__attribute__((weak)) void BasicTimer1_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}

__attribute__((weak)) void BasicTimer2_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}

__attribute__((weak)) void BasicTimer3_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void BasicTimer4_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void BasicTimer5_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}

__attribute__((weak)) void Capture_TIM2_Ch1_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void Capture_TIM2_Ch2_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void Capture_TIM2_Ch3_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void Capture_TIM2_Ch4_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void Capture_TIM3_Ch1_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void Capture_TIM3_Ch2_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void Capture_TIM3_Ch3_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void Capture_TIM3_Ch4_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void Capture_TIM4_Ch1_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void Capture_TIM4_Ch2_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void Capture_TIM4_Ch3_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void Capture_TIM4_Ch4_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void Capture_TIM5_Ch1_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void Capture_TIM5_Ch2_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void Capture_TIM5_Ch3_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
__attribute__((weak)) void Capture_TIM5_Ch4_Callback(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}


/* Esta es la función a la que apunta el sistema en el vector de interrupciones.
 * Se debe utilizar usando exactamente el mismo nombre definido en el vector de interrupciones,
 * Al hacerlo correctamente, el sistema apunta a esta función y cuando la interrupción se lanza
 * el sistema inmediatamente salta a este lugar en la memoria*/
void TIM2_IRQHandler(void){
	/* Limpiamos la bandera que indica que la interrupción se ha generado */
	if (TIM2->SR & TIM_SR_UIF){
			TIM2->SR &= ~TIM_SR_UIF;
			/* LLamamos a la función que se debe encargar de hacer algo con esta interrupción*/
			BasicTimer2_Callback();
		}else if (TIM2->SR & TIM_SR_CC1IF){
			TIM2->SR &= ~TIM_SR_CC1IF;
			TIM2->SR &= ~TIM_SR_CC1OF;
			Capture_TIM2_Ch1_Callback();
		}else if (TIM2->SR & TIM_SR_CC2IF){
			TIM2->SR &= ~TIM_SR_CC2IF;
			TIM2->SR &= ~TIM_SR_CC2OF;
			Capture_TIM2_Ch2_Callback();
		}else if (TIM2->SR & TIM_SR_CC3IF){
			TIM2->SR &= ~TIM_SR_CC3IF;
			TIM2->SR &= ~TIM_SR_CC3OF;
			Capture_TIM2_Ch3_Callback();
		}else if (TIM2->SR & TIM_SR_CC4IF){
			TIM2->SR &= ~TIM_SR_CC4IF;
			TIM2->SR &= ~TIM_SR_CC4OF;
			Capture_TIM2_Ch4_Callback();
		}

}

void TIM3_IRQHandler(void){

	/* Limpiamos la bandera que indica que la interrupción se ha generado */
	if (TIM3->SR & TIM_SR_UIF){
		TIM3->SR &= ~TIM_SR_UIF;
		/* LLamamos a la función que se debe encargar de hacer algo con esta interrupción*/
		BasicTimer3_Callback();
	}else if (TIM3->SR & TIM_SR_CC1IF){
		TIM3->SR &= ~TIM_SR_CC1IF;
		TIM3->SR &= ~TIM_SR_CC1OF;
		Capture_TIM3_Ch1_Callback();
	}else if (TIM3->SR & TIM_SR_CC2IF){
		TIM3->SR &= ~TIM_SR_CC2IF;
		TIM3->SR &= ~TIM_SR_CC2OF;
		Capture_TIM3_Ch2_Callback();
	}else if (TIM3->SR & TIM_SR_CC3IF){
		TIM3->SR &= ~TIM_SR_CC3IF;
		TIM3->SR &= ~TIM_SR_CC3OF;
		Capture_TIM3_Ch3_Callback();
	}else if (TIM3->SR & TIM_SR_CC4IF){
		TIM3->SR &= ~TIM_SR_CC4IF;
		TIM3->SR &= ~TIM_SR_CC4OF;
		Capture_TIM3_Ch4_Callback();
	}
}

void TIM4_IRQHandler(void){

	/* Limpiamos la bandera que indica que la interrupción se ha generado */
	if (TIM4->SR & TIM_SR_UIF){
		TIM4->SR &= ~TIM_SR_UIF;
		/* LLamamos a la función que se debe encargar de hacer algo con esta interrupción*/
		BasicTimer4_Callback();
	}else if (TIM4->SR & TIM_SR_CC1IF){
		TIM4->SR &= ~TIM_SR_CC1IF;
		TIM4->SR &= ~TIM_SR_CC1OF;
		Capture_TIM4_Ch1_Callback();
	}else if (TIM4->SR & TIM_SR_CC2IF){
		TIM4->SR &= ~TIM_SR_CC2IF;
		TIM4->SR &= ~TIM_SR_CC2OF;
		Capture_TIM5_Ch2_Callback();
	}else if (TIM4->SR & TIM_SR_CC3IF){
		TIM4->SR &= ~TIM_SR_CC3IF;
		TIM4->SR &= ~TIM_SR_CC3OF;
		Capture_TIM4_Ch3_Callback();
	}else if (TIM4->SR & TIM_SR_CC4IF){
		TIM4->SR &= ~TIM_SR_CC4IF;
		TIM4->SR &= ~TIM_SR_CC4OF;
		Capture_TIM4_Ch4_Callback();
	}

}

void TIM5_IRQHandler(void){

	/* Limpiamos la bandera que indica que la interrupción se ha generado */
	if (TIM5->SR & TIM_SR_UIF){
		TIM5->SR &= ~TIM_SR_UIF;
		/* LLamamos a la función que se debe encargar de hacer algo con esta interrupción*/
		BasicTimer5_Callback();
	}else if (TIM5->SR & TIM_SR_CC1IF){
		TIM5->SR &= ~TIM_SR_CC1IF;
		TIM5->SR &= ~TIM_SR_CC1OF;
		Capture_TIM5_Ch1_Callback();
	}else if (TIM5->SR & TIM_SR_CC2IF){
		TIM5->SR &= ~TIM_SR_CC2IF;
		TIM5->SR &= ~TIM_SR_CC2OF;
		Capture_TIM5_Ch2_Callback();
	}else if (TIM5->SR & TIM_SR_CC3IF){
		TIM5->SR &= ~TIM_SR_CC3IF;
		TIM5->SR &= ~TIM_SR_CC3OF;
		Capture_TIM5_Ch3_Callback();
	}else if (TIM5->SR & TIM_SR_CC4IF){
		TIM5->SR &= ~TIM_SR_CC4IF;
		TIM5->SR &= ~TIM_SR_CC4OF;
		Capture_TIM5_Ch4_Callback();
	}


}



void startTimer (BasicTimer_Handler_t *ptrTimerConfig){
	ptrTimerConfig->ptrTIMx->CR1 |= TIM_CR1_CEN;
}

void stopTimer (BasicTimer_Handler_t *ptrTimerConfig){
	ptrTimerConfig->ptrTIMx->CR1 &= ~TIM_CR1_CEN;
}


