/*
 * DMA.h
 *
 *  Created on: Sep 12, 2024
 *      Author: juane
 */

#ifndef DMA_H_
#define DMA_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f4xx.h"
#include "I2CDriver.h"


// Define the RX buffer (assuming you want to receive 'data_length' bytes)
#define DATA_LENGTH 1  // Example: 1 bytes

// Define the buffers with alignment for optimal DMA transfers
extern uint8_t rx_buffer __attribute__((aligned(4)));  // Align to 4-byte boundary
extern uint8_t tx_buffer __attribute__((aligned(4)));


// Prioridades para el Free
enum {
	e_DMA_PRIORITY_6 =6,
	e_DMA_PRIORITY_7,
	e_DMA_PRIORITY_8,
	e_DMA_PRIORITY_9,
	e_DMA_PRIORITY_10,
	e_DMA_PRIORITY_11,
	e_DMA_PRIORITY_12,
	e_DMA_PRIORITY_13,
	e_DMA_PRIORITY_14,
	e_DMA_PRIORITY_15
};

typedef struct{
	uint8_t DMA_num;

}DMA_Config_t;

typedef struct{

	DMA_TypeDef        *ptrDMAType;
	DMA_Stream_TypeDef *ptrDMAStream;
	DMA_Config_t        config_DMA;


	// Periferal pointers
	I2C_Handler_t *ptrI2C_handler;

}DMA_Handler_t;

void config_DMA(DMA_Handler_t *ptrDMA_handler[2]);


void DMA1_Stream0_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);

void DMA1_Stream0_Callback_Rx(void);
void DMA1_Stream6_Callback_Tx(void);

#endif /* DMA_H_ */
