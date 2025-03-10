/*
 * AdcDriver.c
 *
 *  Created on: Month XX, 2022
 *      Author: namontoy
 */
#include "AdcDriver.h"
#include "GPIOxDriver.h"

uint16_t	adcRawData = 0;

void adc_Config(ADC_Config_t *adcConfig){

	if (adcConfig->multiChannel == ADC_MULTCH_DISABLE){


			/* 1. Activamos la señal de reloj para el periférico ADC1 (bus APB2)*/

			RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

			// Limpiamos los registros antes de comenzar a configurar
			ADC1->CR1 = 0;
			ADC1->CR2 = 0;

			/* Comenzamos la configuración del ADC1 */
			/* 2. Resolución del ADC */
			switch(adcConfig->resolution){
				case ADC_RESOLUTION_12_BIT:
				{
					ADC1->CR1 &= ~ADC_CR1_RES;

					break;
				}

				case ADC_RESOLUTION_10_BIT:
				{
					ADC1->CR1 |= ADC_CR1_RES_0;
					break;
				}

				case ADC_RESOLUTION_8_BIT:
				{

					ADC1->CR1 |= ADC_CR1_RES_1;
					break;
				}

				case ADC_RESOLUTION_6_BIT:
				{
					ADC1->CR1 |= ADC_CR1_RES;
					break;
				}

				default:
				{	ADC1->CR1 |= ADC_CR1_RES;
					break;
				}
			}

			/* 4. Configuramos el modo Scan como desactivado */
			ADC1->CR1 &= ~ADC_CR1_SCAN;

			/* 5. Configuramos la alineación de los datos (derecha o izquierda) */
			if(adcConfig->dataAlignment == ADC_ALIGNMENT_RIGHT){
				// Alineación a la derecha (esta es la forma "natural")
				ADC1->CR2 &= ~ADC_CR2_ALIGN;
			}
			else{

				// Alineación a la izquierda (para algunos cálculos matemáticos)
				ADC1->CR2 |= ADC_CR2_ALIGN;
			}

			/* 6. Desactivamos el "continuos mode" */

			if (adcConfig->continuosModeEnable == ADC_CONT_DISABLE){
				adc_CONT_OFF();

			}else{
				// Si estamos aqui es porque queremos conversion continua
				adc_CONT_ON();
			}

			/* 7. Acá se debería configurar el sampling...*/
			if(adcConfig->channel <= ADC_CHANNEL_9){
				ADC1->SMPR2 |= (adcConfig->samplingPeriod) << (0x3 * adcConfig->channel);
			}
			else{
				ADC1->SMPR1 |= (adcConfig->samplingPeriod) << (0x3 * adcConfig->channel);
			}

			/* 8. Configuramos la secuencia y cuantos elementos hay en la secuencia */
			// Al hacerlo TODO 0, estamos seleccionando solo 1 elemento en el conteo de la secuencia

			ADC1->SQR1 = 0;

			// Asignamos el canal de la conversión a la primera posición en la secuencia
			ADC1->SQR3 |= (adcConfig->channel << 0);

			/* 9. Configuramos el preescaler del ADC en 2:1 (el mas rápido que se puede tener */
			ADC->CCR |= ADC_CCR_ADCPRE_0;

			if (adcConfig->watchdogs_Enable == ADC_WATCHDOG_ENABLE ){
				// Activamos el watchdogs para todos los canales regulares
				ADC1->CR1 |= ADC_CR1_AWDEN;

				// Activamos las interrupciones por watchdogs
				ADC1->CR1 |= ADC_CR1_AWDIE;

				ADC1->HTR &= 0;

				ADC1->HTR |= adcConfig->threshold_up;

				ADC1->LTR &= 0;

				ADC1->LTR |= adcConfig->threshold_down;

			}else{

				__NOP();

			}


			/* 10. Desactivamos las interrupciones globales */
			__disable_irq();

			/* 11. Activamos la interrupción debida a la finalización de una conversión EOC (CR1)*/
			ADC1->CR1 |= ADC_CR1_EOCIE;


			/* 11a. Matriculamos la interrupción en el NVIC*/
			__NVIC_EnableIRQ(ADC_IRQn);

			/* 11b. Configuramos la prioridad para la interrupción ADC */
			__NVIC_SetPriority(ADC_IRQn, 4);

			/* 12. Activamos el modulo ADC */
			adc_ON();

			/* 13. Activamos las interrupciones globales */
			__enable_irq();

	}else{

		/* 1. Activamos la señal de reloj para el periférico ADC1 (bus APB2)*/

			RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

			// Limpiamos los registros antes de comenzar a configurar
			ADC1->CR1 = 0;
			ADC1->CR2 = 0;

			/* Comenzamos la configuración del ADC1 */
			/* 3. Resolución del ADC */
			switch(adcConfig->resolution){
				case ADC_RESOLUTION_12_BIT:
				{
					ADC1->CR1 &= ~ADC_CR1_RES;

					break;
				}

				case ADC_RESOLUTION_10_BIT:
				{
					ADC1->CR1 |= ADC_CR1_RES_0;
					break;
				}

				case ADC_RESOLUTION_8_BIT:
				{

					ADC1->CR1 |= ADC_CR1_RES_1;
					break;
				}

				case ADC_RESOLUTION_6_BIT:
				{
					ADC1->CR1 |= ADC_CR1_RES;
					break;
				}

				default:
				{	ADC1->CR1 |= ADC_CR1_RES;
					break;
				}
			}

			/* 4. Configuramos el modo Scan como activado */
			ADC1->CR1 |= ADC_CR1_SCAN;

			/* 5. Configuramos la alineación de los datos (derecha o izquierda) */
			if(adcConfig->dataAlignment == ADC_ALIGNMENT_RIGHT){
				// Alineación a la derecha (esta es la forma "natural")
				ADC1->CR2 &= ~ADC_CR2_ALIGN;
			}
			else{

				// Alineación a la izquierda (para algunos cálculos matemáticos)
				ADC1->CR2 |= ADC_CR2_ALIGN;
			}

			/* 6. Desactivamos el "continuos mode" */
			if (adcConfig->continuosModeEnable == ADC_CONT_DISABLE){
				adc_CONT_OFF();

			}else{
				// Si estamos aqui es porque queremos conversion continua
				adc_CONT_ON();
			}

			/* 7. Acá se debería configurar el sampling...*/
			for (uint8_t i = 0; i < adcConfig->numeroDeCanales; i++){
				if(adcConfig->channelVector[i] <= ADC_CHANNEL_9){
					ADC1->SMPR2 |= (adcConfig->samplingPeriod) << (0x3 * adcConfig->channelVector[i]);
				}
				else{
					ADC1->SMPR1 |= (adcConfig->samplingPeriod) << (0x3 * (adcConfig->channelVector[i]- 10));
				}

			}

			/* 8. Configuramos la secuencia y cuantos elementos hay en la secuencia */
			// Al hacerlo todo 0, estamos seleccionando solo 1 elemento en el conteo de la secuencia



			ADC1->SQR1 |= ((adcConfig->numeroDeCanales - 1) << ADC_SQR1_L_Pos);



			for (uint8_t i = 0; i < adcConfig->numeroDeCanales ; i++){
				if (i <= 5){

					ADC1->SQR3 |= (adcConfig->channelVector[i] << 5 * i);

				}else if ((i <= 11) & (i > 5)){

					ADC1->SQR2 |= (adcConfig->channelVector[i] << 5 * (i-6));

				}else if (i <= 15){

					ADC1->SQR1 |= (adcConfig->channelVector[i] << 5 * (i-12));

				}

			}


			//Activamos interrupciones cada fin de secuencia.
			ADC1->CR2 |= ADC_CR2_EOCS;


			/* 9. Configuramos el preescaler del ADC en 2:1 (el mas rápido que se puede tener */
			ADC->CCR |= ADC_CCR_ADCPRE_0;

			if (adcConfig->watchdogs_Enable == ADC_WATCHDOG_ENABLE ){
				// Activamos el watchdogs para todos los canales regulares
				ADC1->CR1 |= ADC_CR1_AWDEN;

				// Activamos las interrupciones por watchdogs
				ADC1->CR1 |= ADC_CR1_AWDIE;

				ADC1->HTR &= 0;

				ADC1->HTR |= adcConfig->threshold_up;

				ADC1->LTR &= 0;

				ADC1->LTR |= adcConfig->threshold_down;

			}else{

				__NOP();

			}


			/* 10. Desactivamos las interrupciones globales */
			__disable_irq();

			/* 11. Activamos la interrupción debida a la finalización de una conversión EOC (CR1)*/
			ADC1->CR1 |= ADC_CR1_EOCIE;

			/* 11a. Matriculamos la interrupción en el NVIC*/
			__NVIC_EnableIRQ(ADC_IRQn);

			/* 11b. Configuramos la prioridad para la interrupción ADC */
			__NVIC_SetPriority(ADC_IRQn, 4);

			/* 12. Activamos el modulo ADC */
			adc_ON();

			/* 13. Activamos las interrupciones globales */
			__enable_irq();



	}
}


/*
 * Esta función habilita la conversion ADC de forma continua o simple.
 * */
void startConvertion(void){

	/* Iniciamos un ciclo de conversión ADC */
	ADC1->CR2 |= ADC_CR2_SWSTART;

}

/*
 * Función que retorna el ultimo dato adquirido por la ADC
 * La idea es que esta función es llamada desde la función callback, de forma que
 * siempre se obtiene el valor mas actual de la conversión ADC.
 * */
uint16_t getADC(void){
	// Esta variable es actualizada en la ISR de la conversión, cada vez que se obtiene
	// un nuevo valor.
	return adcRawData;
}

void adc_ON(void){

	ADC1->CR2 |= ADC_CR2_ADON; // PRENDEMOS EL ADC

}

void adc_OFF(void){

	ADC1->CR2 &= ~ADC_CR2_ADON; // APAGAMOS EL ADC

}

void adc_CONT_ON(void){
	ADC1->CR2 |= ADC_CR2_CONT;

}

void adc_CONT_OFF(void){
	ADC1->CR2 &= ~ADC_CR2_CONT;

}

/*
 * Esta es la ISR de la interrupción por conversión ADC
 */
void ADC_IRQHandler(void){

	if (ADC1->SR & ADC_SR_AWD){

		ADC1->SR &= ~ADC_SR_AWD;

		watchdogs_Callback();
	}

	// Evaluamos que se dio la interrupción por conversión ADC
	if(ADC1->SR & ADC_SR_EOC){
		// Leemos el resultado de la conversión ADC y lo cargamos en una variale auxiliar
		// la cual es utilizada en la función getADC()
		adcRawData = ADC1->DR;

		// Hacemos el llamado a la función que se ejecutará en el main
		adcComplete_Callback();
	}


}

/* Función debil, que debe ser sobreescrita en el main. */
__attribute__ ((weak)) void adcComplete_Callback(void){
	__NOP();
}

__attribute__ ((weak)) void watchdogs_Callback(void){
	__NOP();
}



void adcExternalEXTIConfig(){
	//Seleccionamos el tipo de evento que queremos reconocer en el miltipleXOR, un rising edge.
 	ADC1->CR2 |= ADC_CR2_EXTEN_0;
 	//Activamos los eventos por eventos externos (EXTI11)
	ADC1->CR2 |= (0xF << ADC_CR2_EXTSEL_Pos);
}


void adcTimerEventConfig(){
	//Seleccionamos el tipo de evento que queremos reconocer en el miltipleXOR, un falling edge.
	ADC1->CR2 |= ADC_CR2_EXTEN_1;

	//Activamos los eventos por eventos de PWM (TIMER3 canal 1)
	ADC1->CR2 |= (0x7 << ADC_CR2_EXTSEL_Pos);

}



void adc_Set_Priority(ADC_Config_t *ptrAdcConfig, uint8_t newPriority){

	/* 10. Desactivamos las interrupciones globales */
	__disable_irq();

	/* 11b. Configuramos la prioridad para la interrupción ADC */
	__NVIC_SetPriority(ADC_IRQn, newPriority);

	/* 13. Activamos las interrupciones globales */
	__enable_irq();


}
