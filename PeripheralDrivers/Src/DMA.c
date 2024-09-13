/*
 * DMA.c
 *
 *  Created on: Sep 12, 2024
 *      Author: juane
 */

#include "DMA.h"

void config_DMA(DMA_Handler_t *ptrDMA_handler[2]){

	// 0----> Recepcion, 1-----> Transmisión

	__disable_irq();

	// Limpiamos los buffers en memoria que se usaran para recepcion y transmision de datos
	rx_buffer = 0;
	tx_buffer = 0;

	// activamos el bus AHB1 donde la DMA opera
	if (ptrDMA_handler[0]->ptrDMAType ==DMA1){

		// Predemos el bus correspondiente para el DMA1
		RCC->AHB1ENR = RCC_AHB1ENR_DMA1EN;
	}else if (ptrDMA_handler[0]->ptrDMAType == DMA2){

		// Prednemos el bus correspondiente para el DMA2
		RCC->AHB1ENR = RCC_AHB1ENR_DMA2EN;
	}

	// como por ahora usaremos la DMA para transaccion y recepcion en I2C buscaremos usar el stream 0 para recepcion
	// Y el stream 6 para la transacción, para ambos usando el channel 1.

	///////////////// RX CONFIGURATION///////////////////
	///////STREAM 0

	// Primero desahilitamos el DMA STREAM
	ptrDMA_handler[0]->ptrDMAStream->CR &= ~(DMA_SxCR_EN);

	// Le entregamos a el stream el address the I2C1, el que se usara para la comunicación
	ptrDMA_handler[0]->ptrDMAStream->PAR = (uint32_t) &(ptrDMA_handler[0]->ptrI2C_handler->ptrI2Cx->DR);

	// Definimos la direccion de memoria del buffer que recibira los datos correspondientes
	ptrDMA_handler[0]->ptrDMAStream->M0AR = (uint32_t)rx_buffer;

	// Lo que sigue es definir en el registro NDTR el largo de los datos en bytes, en nuestro caso, nuestro buffer de recepcion es 4 bytes de largo
	ptrDMA_handler[0]->ptrDMAStream->NDTR = DATA_LENGTH;

	// Seteamos la prioridad del stream
	ptrDMA_handler[0]->ptrDMAStream->CR |= (0b10 << 16); // elegimos para la recepcion una prioridad High

	//Seleccionamos el canal 1 para I2C1 recepción

	ptrDMA_handler[0]->ptrDMAStream->CR |= (0b001 << 25); // 0b001 es para el canal 1

	// Configuramos la direccion de transaccion de periferal to memory
	ptrDMA_handler[0]->ptrDMAStream->CR &= ~(0b11 << 6);

	// seteamos el modod de incremento del puntero que apunta a los datos en recepción
	ptrDMA_handler[0]->ptrDMAStream->CR |= (0b1 << 10);

	// Seteamos el tamaño de los datos de memoria y del periferico, ambos de 8 bits
	ptrDMA_handler[0]->ptrDMAStream->CR &= ~(0b11 << 13);  // Peripheral size = 8 bits
	ptrDMA_handler[0]->ptrDMAStream->CR &= ~(0b11 << 11);  // Memory size = 8 bits


	///////////////// TX CONFIGURATION///////////////////
	///////STREAM 6

	// Primero desahilitamos el DMA STREAM
	ptrDMA_handler[1]->ptrDMAStream->CR &= ~(DMA_SxCR_EN);

	// Le entregamos a el stream el address the I2C1, el que se usara para la comunicación
	ptrDMA_handler[1]->ptrDMAStream->PAR = (uint32_t) &(ptrDMA_handler[1]->ptrI2C_handler->ptrI2Cx->DR);

	// Definimos la direccion de memoria del buffer que transmitirá los datos correspondientes
	ptrDMA_handler[1]->ptrDMAStream->M0AR = (uint32_t)tx_buffer;

	// Lo que sigue es definir en el registro NDTR el largo de los datos en bytes, en nuestro caso, nuestro buffer de transmisión es 4 bytes de largo
	ptrDMA_handler[1]->ptrDMAStream->NDTR = DATA_LENGTH;

	//Seleccionamos el canal 1 para I2C1 transmisión
	ptrDMA_handler[1]->ptrDMAStream->CR |= (0b001 << 25); // 0b001 es para el canal 1

	// Configuramos la direccion de transaccion de memory to peripheral
	ptrDMA_handler[1]->ptrDMAStream->CR |= (0b01 << 6);

	// seteamos el modo de incremento del puntero que apunta a los datos en recepción
	ptrDMA_handler[1]->ptrDMAStream->CR |= (0b1 << 10);

	// Seteamos el tamaño de los datos de memoria y del periferico, ambos de 8 bits
	ptrDMA_handler[1]->ptrDMAStream->CR &= ~(0b11 << 13);  // Peripheral size = 8 bits
	ptrDMA_handler[1]->ptrDMAStream->CR &= ~(0b11 << 11);  // Memory size = 8 bits


	// En esta parte vamos a activar la interrupcion de Tranfer Complete Interrupt Enable, para saber la completición de mandado de datos
	// tando transmitiendo y recibiendo
	ptrDMA_handler[1]->ptrDMAStream->CR |= (0b1 << 4); // habilitamos la interrupcion TCIE


	// Habilitamos el stream
	ptrDMA_handler[0]->ptrDMAStream->CR |= (0b1 << 0);  // Enable the stream
	ptrDMA_handler[1]->ptrDMAStream->CR |= (0b1 << 0);  // Enable the stream


	//Habilitamos en el NVIC la interrupcion de TCI
	// Enable DMA1_Stream0 interrupt (for RX)
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	NVIC_SetPriority(DMA1_Stream0_IRQn, 1);  // Set priority if needed

	// Enable DMA1_Stream6 interrupt (for TX)
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	NVIC_SetPriority(DMA1_Stream6_IRQn, 1);  // Set priority if needed

	// Activamos de nuevo las interrupciones globales
	__enable_irq();
}


// Se completo la interrupcion de Recepcion , se analiza la bandera correspondiente a la interrupcion dada y luego se baja
void DMA1_Stream0_IRQHandler(void)
{
    if (DMA1->LISR & (0b1 << 5))  // Transfer Complete flag for Stream 0
    {
        DMA1->LIFCR |= (0b1 << 5);  // Clear the flag
        // Handle RX transfer complete (process the received data)
        DMA1_Stream0_Callback_Rx();
    }
}

void DMA1_Stream6_IRQHandler(void)
{
    if (DMA1->HISR & (1 << 21))  // Transfer Complete flag for Stream 6
    {
        DMA1->HIFCR |= (1 << 21);  // Clear the flag
        // Handle TX transfer complete (clean up or signal end of transmission)
        DMA1_Stream6_Callback_Tx();
    }
}


__attribute__((weak)) void DMA1_Stream0_Callback_Rx(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}

__attribute__((weak)) void DMA1_Stream6_Callback_Tx(void){
	  /* NOTE : This function should not be modified, when the callback is needed,
	            the BasicTimerX_Callback could be implemented in the main file
	   */
	__NOP();
}
