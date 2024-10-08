/*
 * USARTxDriver.c
 *
 *  Created on: Apr 6, 2022
 *      Author: namontoy
 */

#include <stm32f4xx.h>
#include "USARTxDriver.h"
#include <math.h>


uint16_t mant_DIVfrac  = 0;
uint16_t mant          = 0;
float DIVfrac          = 0;
double value            = 0;

/**
 * Configurando el puerto Serial...
 * Recordar que siempre se debe comenzar con activar la señal de reloj
 * del periferico que se está utilizando.
 */
void USART_Config(USART_Handler_t *ptrUsartHandler){

	__disable_irq();

	/* 1. Activamos la señal de reloj que viene desde el BUS al que pertenece el periferico */
	/* Lo debemos hacer para cada uno de las posibles opciones que tengamos (USART1, USART2, USART6) */
    /* 1.1 Configuramos el USART1 */
	if(ptrUsartHandler->ptrUSARTx == USART1){
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	}
	else if(ptrUsartHandler->ptrUSARTx == USART2){
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	}
	
    /* 1.2 Configuramos el USART6 */
	else if(ptrUsartHandler->ptrUSARTx == USART6){
		RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
	}
	else{
		__NOP();
	}
	/* 2. Configuramos el tamaño del dato, la paridad y los bit de parada */
	/* En el CR1 estan parity (PCE y PS) y tamaño del dato (M) */
	/* Mientras que en CR2 estan los stopbit (STOP)*/
	/* Configuracion del Baudrate (registro BRR) */
	/* Configuramos el modo: only TX, only RX, o RXTX */
	/* Por ultimo activamos el modulo USART cuando todo esta correctamente configurado */

	// 2.1 Comienzo por limpiar los registros, para cargar la configuración desde cero
	ptrUsartHandler->ptrUSARTx->CR1 = 0;
	ptrUsartHandler->ptrUSARTx->CR2 = 0;

	// 2.2 Configuracion del Parity:
	// Verificamos si el parity esta activado o no
    // Tenga cuidado, el parity hace parte del tamaño de los datos...
	if(ptrUsartHandler->USART_Config.USART_parity != USART_PARITY_NONE){

		// Verificamos si se ha seleccionado ODD or EVEN
		if(ptrUsartHandler->USART_Config.USART_parity == USART_PARITY_EVEN){
			// Es even, entonces cargamos la configuracion adecuada
			ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_PS;
			ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_PEIE;
			
		}else{
			// Si es "else" significa que la paridad seleccionada es ODD, y cargamos esta configuracion
			ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_PS;
			ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_PEIE;
		}
	}else{
		// Si llegamos aca, es porque no deseamos tener el parity-check
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_PCE ;
	}

	// 2.3 Configuramos el tamaño del dato
	if(ptrUsartHandler->USART_Config.USART_datasize == USART_DATASIZE_8BIT){
			// Son 8 bits, entonces cargamos la configuracion adecuada
			ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_M;

	}else{
			// Si es "else" significa que se trata de un tamaño de dato de 9 bits, y cargamos esta configuracion
			ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_M;
	}

	// 2.4 Configuramos los stop bits (SFR USART_CR2)
	switch(ptrUsartHandler->USART_Config.USART_stopbits){
		case USART_STOPBIT_1: {
			// Debemos cargar el valor 0b00 en los dos bits de STOP
			ptrUsartHandler->ptrUSARTx->CR2 &= ~USART_CR2_STOP;
			break;
		}
		case USART_STOPBIT_0_5: {
			// Debemos cargar el valor 0b01 en los dos bits de STOP
			ptrUsartHandler->ptrUSARTx->CR2 |= USART_CR2_STOP & (~USART_CR2_STOP_1);
			break;
		}
		case USART_STOPBIT_2: {
			// Debemoscargar el valor 0b10 en los dos bits de STOP
			ptrUsartHandler->ptrUSARTx->CR2 |= USART_CR2_STOP & (~USART_CR2_STOP_0);
			break;
		}
		case USART_STOPBIT_1_5: {
			// Debemoscargar el valor 0b11 en los dos bits de STOP
			ptrUsartHandler->ptrUSARTx->CR2 |= USART_CR2_STOP;
			break;
		}
		default: {
			// En el caso por defecto seleccionamos 1 bit de parada
			ptrUsartHandler->ptrUSARTx->CR2 &= ~USART_CR2_STOP;
			break;
		}
	}

		// 2.5 Configuracion del Baudrate (SFR USART_BRR)
		ptrUsartHandler->ptrUSARTx->BRR = brrCalculus (ptrUsartHandler, ptrUsartHandler->USART_Config.USART_MCUvelocity);

//     	if (ptrUsartHandler->USART_Config.USART_MCUvelocity == USART_16MHz_VELOCITY){
//		// 2.5 Configuracion del Baudrate (SFR USART_BRR)
//		// Ver tabla de valores (Tabla 73), Frec = 16MHz, overr = 0;
//		if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_9600){
//			// El valor a cargar es 104.1875 -> Mantiza = 104,fraction = 0.1875
//			// Mantiza = 104 = 0x68, fraction = 16 * 0.1875 = 3
//			// Valor a cargar 0x0683
//			// Configurando el Baudrate generator para una velocidad de 9600bps
//			ptrUsartHandler->ptrUSARTx->BRR = 0x0683;
//		}
//
//		else if (ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_19200) {
//			// El valor a cargar es 52.0625 -> Mantiza = 52,fraction = 0.0625
//			// Mantiza = 52 = 0x34, fraction = 16 * 0.0 = 1
//			//Configurando el Baudrate generator para una velocidad de 19200bps
//			ptrUsartHandler->ptrUSARTx->BRR = 0x0341;
//		}
//
//		else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_28800){
//			// El valor a cargar es 34,7222 -> Mantiza = 34, fraction = 0.7222
//			// Mantiza = 34 = 0x22, fraction = 16 * 0.7222 = 11,55 = C
//			//Configurando el Baudrate generator para una velocidad de 115200bps
//			ptrUsartHandler->ptrUSARTx->BRR = 0x22C;
//
//		}
//
//		else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_115200){
//			// El valor a cargar es 8.6875 -> Mantiza = 8,fraction = 0.6875
//			// Mantiza = 8 = 0x8, fraction = 16 * 0.6875 = 11
//			//Configurando el Baudrate generator para una velocidad de 115200bps
//			ptrUsartHandler->ptrUSARTx->BRR = 0x08B;
//
//		}else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_CUSTOM_USART1_6){
//			// El valor a cargar es 54,2538 -> Mantiza = 54,fraction = 0.2538
//			// Mantiza = 52 = 0x34, fraction = 16 * 0.2538 = 4
//			//Configurando el Baudrate generator para una velocidad de 115200bps
//			ptrUsartHandler->ptrUSARTx->BRR = 0x344;
//
//		}else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_CUSTOM_USART2){
//			// El valor a cargar es 27,1267 -> Mantiza = 27,fraction = 0.1267
//			// Mantiza = 27 = 0x1B, fraction = 16 * 0.1267 = 2
//			//Configurando el Baudrate generator para una velocidad de 115200bps
//			ptrUsartHandler->ptrUSARTx->BRR = 0x1B2;
//		}
//	}else if (ptrUsartHandler->USART_Config.USART_MCUvelocity == USART_100MHz_VELOCITY){
//
//		// 2.5 Configuracion del Baudrate (SFR USART_BRR)
//		// Ver tabla de valores (Tabla 73), Frec = 100MHz, overr = 0;
//		if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_9600){
//			// El valor a cargar es 651.0417 -> Mantiza = 651,fraction = 0.0417
//			// Mantiza = 651 = 0x28B, fraction = 16 * 0.0417 = 1
//			// Valor a cargar 0x28B1
//			// Configurando el Baudrate generator para una velocidad de 9600bps
//			ptrUsartHandler->ptrUSARTx->BRR = 0x28B1;
//		}
//
//		else if (ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_19200) {
//			// El valor a cargar es 325,5208 -> Mantiza = 325,fraction = 0.5208
//			// Mantiza = 325 = 0x145, fraction = 16 * 0.5208 = 8
//			//Configurando el Baudrate generator para una velocidad de 19200bps
//			ptrUsartHandler->ptrUSARTx->BRR = 0x1458;
//		}
//
//		else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_28800){
//			// El valor a cargar es 217.0139 -> Mantiza = 217, fraction = 0.0139
//			// Mantiza = 217 = 0xD9, fraction = 16 * 0.0139 = 0 = 0
//			//Configurando el Baudrate generator para una velocidad de 115200bps
//			ptrUsartHandler->ptrUSARTx->BRR = 0x0D90;
//
//		}
//
//		else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_115200){
//			// El valor a cargar es 54.2535 -> Mantiza = 54,fraction = 0.2535
//			// Mantiza = 54 = 0x36, fraction = 16 * 0.2535 = 4
//			//Configurando el Baudrate generator para una velocidad de 115200bps
//			ptrUsartHandler->ptrUSARTx->BRR = 0x0364;
//
//		}else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_CUSTOM_USART1_6){
//			// El valor a cargar es 54,2538 -> Mantiza = 54,fraction = 0.2538
//			// Mantiza = 52 = 0x34, fraction = 16 * 0.2538 = 4
//			//Configurando el Baudrate generator para una velocidad de 115200bps
//			ptrUsartHandler->ptrUSARTx->BRR = 0x344;
//
//		}else if(ptrUsartHandler->USART_Config.USART_baudrate == USART_BAUDRATE_CUSTOM_USART2){
//			// El valor a cargar es 27,1267 -> Mantiza = 27,fraction = 0.1267
//			// Mantiza = 27 = 0x1B, fraction = 16 * 0.1267 = 2
//			//Configurando el Baudrate generator para una velocidad de 115200bps
//			ptrUsartHandler->ptrUSARTx->BRR = 0x1B2;
//		}
//
//
//
//
//	}

	// 2.6 Configuramos el modo: TX only, RX only, RXTX, disable
	switch(ptrUsartHandler->USART_Config.USART_mode){
		case USART_MODE_TX:
		{
			// Activamos la parte del sistema encargada de enviar
			ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TE;
			break;
		}
		case USART_MODE_RX:
		{
			// Activamos la parte del sistema encargada de recibir
			ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_RE;
			break;
		}
		case USART_MODE_RXTX:
		{
			// Activamos ambas partes, tanto transmision como recepcion
			ptrUsartHandler->ptrUSARTx->CR1 |= (USART_CR1_TE | USART_CR1_RE);
			break;
		}
		case USART_MODE_DISABLE:
		{
			// Desactivamos ambos canales
			ptrUsartHandler->ptrUSARTx->CR1 &= ~(USART_CR1_TE | USART_CR1_RE);
			break;
		}

		default:
		{
			// Actuando por defecto, desactivamos ambos canales
			ptrUsartHandler->ptrUSARTx->CR1 &= ~(USART_CR1_TE | USART_CR1_RE);
			break;
		}
	}

	// 2.7 Activamos el modulo serial.
	if(ptrUsartHandler->USART_Config.USART_mode != USART_MODE_DISABLE){
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_UE;
	}else{
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_UE;
	}

	//Habilitamos las interrupciones de recepcion RXNEIE
	if (ptrUsartHandler->USART_Config.USART_enableInRx == USART_INTERRUPT_RX_ENABLE){
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_RXNEIE;

		if(ptrUsartHandler->ptrUSARTx == USART1){
					// Activando en NVIC para la interrupción del USART1
					__NVIC_EnableIRQ(USART1_IRQn);
					__NVIC_SetPriority(USART1_IRQn, 1);
		}
		else if(ptrUsartHandler->ptrUSARTx == USART2){
					// Activando en NVIC para la interrupción del USART2
					__NVIC_EnableIRQ(USART2_IRQn);
					__NVIC_SetPriority(USART2_IRQn, 1);
		}
		else if(ptrUsartHandler->ptrUSARTx == USART6){
				// Activando en NVIC para la interrupción del USART6
					__NVIC_EnableIRQ(USART6_IRQn);
					__NVIC_SetPriority(USART6_IRQn, 1);
		}
		else{
				__NOP();
		}
	}else{
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_RXNEIE;
	}

	//Habilitamos las interrupciones de  Transmisión
	if (ptrUsartHandler->USART_Config.USART_enableInTx == USART_INTERRUPT_TX_ENABLE){
		ptrUsartHandler->ptrUSARTx->CR1 |= USART_CR1_TXEIE;;

		if(ptrUsartHandler->ptrUSARTx == USART1){
					// Activando en NVIC para la interrupción del USART1
					__NVIC_EnableIRQ(USART1_IRQn);
		}
		else if(ptrUsartHandler->ptrUSARTx == USART2){
					// Activando en NVIC para la interrupción del USART2
					__NVIC_EnableIRQ(USART2_IRQn);
		}
		else if(ptrUsartHandler->ptrUSARTx == USART6){
				// Activando en NVIC para la interrupción del USART6
					__NVIC_EnableIRQ(USART6_IRQn);
		}
		else{
				__NOP();
		}
	}else{
		ptrUsartHandler->ptrUSARTx->CR1 &= ~USART_CR1_TXEIE;
	}

	__enable_irq();
}



uint32_t brrCalculus (USART_Handler_t *ptrUsartHandler, uint32_t MCUvelocity){


	switch(ptrUsartHandler->USART_Config.USART_baudrate){

	case USART_BAUDRATE_9600:{

		 value = 1/(16.0 * 9600);

		 value = ((float) MCUvelocity ) *value;

		 mant = (int) value;

		 DIVfrac = value -(int) value;

		 DIVfrac = roundToNDecimals(DIVfrac, 4 );

		 DIVfrac = 16*DIVfrac;

		 DIVfrac = round(DIVfrac);

		 mant_DIVfrac = (mant << 4) | ((unsigned int) DIVfrac);


		break;
	}
	case USART_BAUDRATE_19200:{

		 value = 1/(16.0 * 19200);

		 value = ((float) MCUvelocity ) *value;

		 mant = (int) value;

		 DIVfrac = value -(int) value;

		 DIVfrac = roundToNDecimals(DIVfrac, 4 );

		 DIVfrac = 16*DIVfrac;

		 DIVfrac = round(DIVfrac);

		 mant_DIVfrac = (mant << 4) | ((unsigned int) DIVfrac);


		break;
	}
	case USART_BAUDRATE_28800:{

		 value = 1/(16.0 * 28800);

		 value = ((float) MCUvelocity ) *value;

		 mant = (int) value;

		 DIVfrac = value -(int) value;

		 DIVfrac = roundToNDecimals(DIVfrac, 4 );

		 DIVfrac = 16*DIVfrac;

		 DIVfrac = round(DIVfrac) + 1;

		 mant_DIVfrac = (mant << 4) | ((unsigned int) DIVfrac);


		break;
	}
	case USART_BAUDRATE_115200:{

		 value = 1/(16.0 * 115200);

		 value = ((float) MCUvelocity ) *value;

		 mant = (int) value;

		 DIVfrac = value -(int) value;

		 DIVfrac = roundToNDecimals(DIVfrac, 4 );

		 DIVfrac = 16*DIVfrac;

		 DIVfrac = round(DIVfrac);

		 mant_DIVfrac = (mant << 4) | ((unsigned int) DIVfrac);


		break;
	}
	default:{

		__NOP();
		break;
	}

	}


	return mant_DIVfrac;
}


/* funcion para escribir un solo char */
void writeChar(USART_Handler_t *ptrUsartHandler, int dataToSend ){
	while( !(ptrUsartHandler->ptrUSARTx->SR & USART_SR_TXE)){
		__NOP();
	}

	ptrUsartHandler->ptrUSARTx->DR = dataToSend;

}


void writeMsg(USART_Handler_t *ptrUsartHandler, const char* msgToSend){

	while(*msgToSend != '\0'){
		writeChar(ptrUsartHandler, *msgToSend);
		msgToSend ++ ;
	}
}

__attribute__((weak))	void usart2Rx_Callback(void){
	__NOP();
}
__attribute__((weak))	void usart1Rx_Callback(void){
	__NOP();
}
__attribute__((weak))	void usart6Rx_Callback(void){
	__NOP();
}

uint8_t auxRxData = 0;

uint8_t getRxData(void){
	return auxRxData;
}

void USART2_IRQHandler(void){

	if(USART2->SR & USART_SR_RXNE){
		auxRxData = (uint8_t) USART2->DR;
		usart2Rx_Callback();
	}

}
//	else{
//		USART2->SR &= ~(USART_SR_PE);
//		writeMsg(ptrUSARTHandler, "Mensaje corrompido, enviar de nuevo");
//	}
//	if(USART2->SR & USART_SR_TXE){
//		usart2Rx_Callback();
//	}

void USART1_IRQHandler(void){


	if(USART1->SR & USART_SR_RXNE){
		auxRxData = (uint8_t) USART1->DR;
		usart1Rx_Callback();
	}
}
//	else{
//		USART1->SR &= ~(USART_SR_PE);
//		writeMsg(ptrUSARTHandler, "Mensaje corrompido, enviar de nuevo");
//	}
//	if(USART1->SR & USART_SR_TXE){
//		usart1Rx_Callback();
//	}



void USART6_IRQHandler(void){

	if(USART6->SR & USART_SR_RXNE){
		auxRxData = (uint8_t) USART6->DR;
		usart6Rx_Callback();
	}

}


float roundToNDecimals(float number, int n) {

    double factor = pow(10, n);

    number *= factor;

    number = round(number);

    number /= factor;

    return number;
}

void usart_Set_Priority(USART_Handler_t *ptrUsartHandler, uint8_t newPriority){

	__disable_irq();


	if(ptrUsartHandler->ptrUSARTx == USART1){
				// Seteamos la prioridad en NVIC para la interrupción del USART1
		__NVIC_SetPriority(USART1_IRQn, newPriority);
	}
	else if(ptrUsartHandler->ptrUSARTx == USART2){
				// Seteamos la prioridad en NVIC para la interrupción del USART2
		__NVIC_SetPriority(USART2_IRQn, newPriority);
	}
	else if(ptrUsartHandler->ptrUSARTx == USART6){
			// Seteamos la prioridad en NVIC para la interrupción del USART6
		__NVIC_SetPriority(USART6_IRQn, newPriority);
	}
	else{
			__NOP();
	}



	__enable_irq();

}

//	else{
//		USART6->SR &= ~(USART_SR_PE);
//		writeMsg(ptrUSARTHandler, "Mensaje corrompido, enviar de nuevo");
//	}
//	if(USART6->SR & USART_SR_TXE){
//		usart6Rx_Callback();
//	}


/*
 * Hola Juan Esteban, tu solución no es del todo correcta, intentaste tener en cuenta la diferente velocidad de los buses APB1 y APB2,
 * pero no está escrito de forma correcta. Algunas observaciones generales:
 * - Tienes parcialmente en cuenta que el bus APB1 (USART1 y USART6) corre mas rapido que el APB2 (USART2), por lo cual toda tu
 *   configuracion es parcial y muy confusa.
 *   El nombre que le pusiste al driver para cambiar el reloj principal es realmente complejo y no muy claro... no entiendo que es
 *   ese "Hun" que pones allí.
 *
 *   Calificación = 3.8
 *
 * */
