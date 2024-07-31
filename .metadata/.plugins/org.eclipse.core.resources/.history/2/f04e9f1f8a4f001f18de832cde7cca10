/*
 * USARTxDriver.h
 *
 *  Created on: Apr 6, 2022
 *      Author: namontoy
 */

#include <stdio.h>
#include "stm32f4xx.h"

#ifndef INC_USARTXDRIVER_H_
#define INC_USARTXDRIVER_H_

#define USART_MODE_TX		0
#define USART_MODE_RX		1
#define USART_MODE_RXTX		2
#define USART_MODE_DISABLE	3

#define USART_BAUDRATE_9600		0
#define USART_BAUDRATE_19200	        1
#define USART_BAUDRATE_28800	        2
#define USART_BAUDRATE_115200	        3
#define USART_BAUDRATE_CUSTOM_USART2	4
#define USART_BAUDRATE_CUSTOM_USART1_6  5

#define USART_DATASIZE_8BIT		0
#define USART_DATASIZE_9BIT		1

#define USART_PARITY_NONE	0
#define USART_PARITY_ODD	1
#define USART_PARITY_EVEN	2

#define USART_STOPBIT_1		0
#define USART_STOPBIT_0_5	1
#define USART_STOPBIT_2		2
#define USART_STOPBIT_1_5	3

#define USART_INTERRUPT_RX_DISABLE  0
#define USART_INTERRUPT_RX_ENABLE   1

#define USART_INTERRUPT_TX_DISABLE  0
#define USART_INTERRUPT_TX_ENABLE   1

#define USART_16MHz_VELOCITY   16000000
#define USART_50MHz_VELOCITY   50000000
#define USART_100MHz_VELOCITY  100000000

/* Estructura para la configuración de la comunicacion:
 * Velocidad (baudrate)
 * Tamaño de los datos
 * Control de errores
 * Bit de parada
 */
typedef struct
{
	uint8_t USART_enableInTx;
	uint8_t USART_enableInRx;
	uint8_t USART_mode;
	uint8_t USART_baudrate;
	uint8_t USART_datasize;
	uint8_t USART_parity;
	uint8_t USART_stopbits;
	uint8_t USART_parityError;
	uint32_t USART_MCUvelocity;

}USART_Config_t;

/*
 * Definicion del Handler para un USART:
 * - Estructura que contiene los SFR que controlan el periferico
 * - Estructura que contiene la configuración especifica del objeto
 * - Buffer de recepcion de datos
 * - Elemento que indica cuantos datos se recibieron
 * - Buffer de transmision de datos
 * - Elemento que indica cuantos datos se deben enviar.
 */
typedef struct
{
	USART_TypeDef	*ptrUSARTx;
	USART_Config_t	USART_Config;
	uint8_t			receptionBuffer[64];
	uint8_t			dataInputSize;
	uint8_t			transmisionBuffer[64];
	uint8_t			dataOutputSize;

}USART_Handler_t;



/* Definicion de los prototipos para las funciones del USART */
void USART_Config(USART_Handler_t *ptrUsartHandler);
void writeChar(USART_Handler_t *ptrUsartHandler, int dataToSend );
void writeMsg(USART_Handler_t *ptrUsartHandler, char* msgToSend);
uint32_t brrCalculus (USART_Handler_t *ptrUsartHandler, uint32_t MCUvelocity);
uint8_t getRxData(void);
float roundToNDecimals(float number, int n);
void usart1Rx_Callback(void);
void usart2Rx_Callback(void);
void usart6Rx_Callback(void);


#endif /* INC_USARTXDRIVER_H_ */
