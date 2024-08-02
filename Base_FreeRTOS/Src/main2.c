/*
 * main2.c
 *
 *  Created on: Aug 2, 2024
 *      Author: juan
 */


#include <stm32f4xx.h>


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "PwmDriver.h"
#include "EXTIDriver.h"
#include "RCCHunMHz.h"
#include "USARTxDriver.h"

/*definicion de variables del sistema*/


#define STACK_SIZE 200

uint32_t SystemCoreClock = 100000000;

BaseType_t xReturned;
/*Cabecera de la funcion de tarea 1 */
void vTask_Menu( void * pvParameters );
void vTask_Print( void * pvParameters );
void vTask_Commands( void * pvParameters );


TaskHandle_t xHandleTask_Menu     = NULL;
TaskHandle_t xHandleTask_Print    = NULL;
TaskHandle_t xHandleTask_Commands = NULL;

QueueHandle_t xQueue_Print;
QueueHandle_t xQueue_InputData;

void inSystem (void);
void parseCommands(char *stringVector);

//Definición Handlers
//GPIO
GPIO_Handler_t handlerPinA5           = {};
//GPIO_Handler_t handlerUserButton      = {};
GPIO_Handler_t handlerUSART_RX       = {};
GPIO_Handler_t handlerUSART_TX       = {};

//Extis
EXTI_Config_t  handler_exti_userButon = {};

//USART
USART_Handler_t handlerUSART2 = {};

//Timers
BasicTimer_Handler_t handlerTimerBlinky = {};

// Variables para los comandos
char bufferReception[64];
uint8_t counterReception = 0;
uint8_t doneTransaction = RESET;
uint8_t rxData = '\0';
char cmd[32];

typedef struct{
	uint8_t payload[10];
	uint32_t len;

}command_t;

typedef enum {
	sMainMenu = 0,
	sLedEffect,
	sRtcMenu
}state_t;

unsigned int firstParameter;
unsigned int secondParameter;
unsigned int thirdParameter;
char userMsg[64];

int main(void)
{


	//Activamos el FPU o la unidad de punto flotante
	SCB -> CPACR |= (0xF << 20);


	//Activamos el contador
   	DWT -> CTRL    |= (1 << 0);

	// Configuracion de orden de prioridad
	//vInitPrioGroupValue();
//
//	/* Primero configuramos */
//	SEGGER_SYSVIEW_Conf();
//	/* Despues activamos el sistema */
//	SEGGER_SYSVIEW_Start();


	inSystem ();

	xReturned = xTaskCreate(
						vTask_Menu,       /* Function that implements the task. */
						"Task-MENU",          /* Text name for the task. */
						STACK_SIZE,      /* Stack size in words, not bytes. */
						NULL,    /* Parameter passed into the task. */
						3,/* Priority at which the task is created. */
						&xHandleTask_Menu );      /* Used to pass out the created task's handle. */


	 configASSERT( xReturned == pdPASS );

	xReturned = xTaskCreate(
						vTask_Print,       /* Function that implements the task. */
						"Task-Print",          /* Text name for the task. */
						STACK_SIZE,      /* Stack size in words, not bytes. */
						NULL,    /* Parameter passed into the task. */
						3,/* Priority at which the task is created. */
						&xHandleTask_Print );      /* Used to pass out the created task's handle. */


	 configASSERT( xReturned == pdPASS );


	xReturned = xTaskCreate(
						vTask_Commands,       /* Function that implements the task. */
						"Task-Commands",          /* Text name for the task. */
						STACK_SIZE,      /* Stack size in words, not bytes. */
						NULL,    /* Parameter passed into the task. */
						3,/* Priority at which the task is created. */
						&xHandleTask_Commands );      /* Used to pass out the created task's handle. */


	 configASSERT( xReturned == pdPASS );

//	 xReturned = xTaskCreate(
//	 	                    vTaskTwo,       /* Function that implements the task. */
//	 	                    "Task-Button",          /* Text name for the task. */
//	 	                    STACK_SIZE,      /* Stack size in words, not bytes. */
//	 	                    "HOLA MUNDO",    /* Parameter passed into the task. */
//	 	                    3,/* Priority at which the task is created. */
//	 	                    &xHandleTask_Button );      /* Used to pass out the created task's handle. */


	 //Creacion de colas

	 xQueue_InputData = xQueueCreate(10,sizeof(char));
	 configASSERT(xQueue_InputData != NULL);// Verificamos que se creo la cola correctamente

	 //XQueue_Print = xQueueCreate (10, sizeof (struct AMessage *))
	 xQueue_Print = xQueueCreate(10,sizeof(size_t));
	 configASSERT(xQueue_Print != NULL); // Verificamos que se creo la cola correctamente

	 //Creando el timer de FreeRTOS


	 /* Start the created tasks running. */
	 vTaskStartScheduler();


    /* Loop forever */
	while(1){
//		GPIOxTooglePin(&handlerPinA5);
//		for (uint16_t steps = 0 ; steps < 10000 ;steps++){
//
//		}


//		if (rxData != '\0'){
//			bufferReception[counterReception] = rxData;
//			counterReception++;
//
//			if (rxData == '@'){
//				doneTransaction = SET;
//
//				bufferReception[counterReception] = '\0';
//
//				counterReception = 0;
//
//			}
//
//			rxData = '\0';
//
//		}
//
//		if (doneTransaction){
//			parseCommands(bufferReception);
//			doneTransaction = RESET;
//		}
//
		//Si se llega hasta aca es porque algo salio mal

	}
}


void inSystem (void){




	//Descripcion de la configuracion

	// Activamos la maxima velocidad del microcontrolador
	RCC_enableMaxFrequencies();

	//BLINKY LED
	handlerPinA5.pGPIOx = GPIOA;
	handlerPinA5.GPIO_PinConfig.GPIO_PinAltFunMode = AF0;
	handlerPinA5.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	handlerPinA5.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerPinA5.GPIO_PinConfig.GPIO_PinNumber = PIN_5;
	handlerPinA5.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinA5.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinA5);
	GPIO_WritePin(&handlerPinA5, RESET);

	//TIMER Blinky

	handlerTimerBlinky.ptrTIMx                           = TIM3;
	handlerTimerBlinky.TIMx_Config.TIMx_interruptEnable  = BTIMER_ENABLE_INTERRUPT;
	handlerTimerBlinky.TIMx_Config.TIMx_mode             = BTIMER_MODE_UP;
	handlerTimerBlinky.TIMx_Config.TIMx_speed            = BTIMER_SPEED_100MHz_100us;
	handlerTimerBlinky.TIMx_Config.TIMx_period           = 250;
	BasicTimer_Config(&handlerTimerBlinky);
	startTimer(&handlerTimerBlinky);


	//USART 2 Comunicacion serial
	handlerUSART_RX.pGPIOx = GPIOA;
	handlerUSART_RX.GPIO_PinConfig.GPIO_PinAltFunMode = AF7;
	handlerUSART_RX.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	handlerUSART_RX.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerUSART_RX.GPIO_PinConfig.GPIO_PinNumber = PIN_3;
	handlerUSART_RX.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerUSART_RX.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerUSART_RX);


	handlerUSART_TX.pGPIOx = GPIOA;
	handlerUSART_TX.GPIO_PinConfig.GPIO_PinAltFunMode = AF7;
	handlerUSART_TX.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	handlerUSART_TX.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerUSART_TX.GPIO_PinConfig.GPIO_PinNumber = PIN_2;
	handlerUSART_TX.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerUSART_TX.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerUSART_TX);

	handlerUSART2.ptrUSARTx                      = USART2;
	handlerUSART2.USART_Config.USART_MCUvelocity = USART_50MHz_VELOCITY;
	handlerUSART2.USART_Config.USART_baudrate    = USART_BAUDRATE_19200;
	handlerUSART2.USART_Config.USART_enableInRx  = USART_INTERRUPT_RX_ENABLE;
	handlerUSART2.USART_Config.USART_enableInTx  = USART_INTERRUPT_TX_DISABLE;
	handlerUSART2.USART_Config.USART_mode        = USART_MODE_RXTX;
	handlerUSART2.USART_Config.USART_parity      = USART_PARITY_NONE;
	handlerUSART2.USART_Config.USART_stopbits    = USART_STOPBIT_1;
	handlerUSART2.USART_Config.USART_datasize    = USART_DATASIZE_8BIT;
	USART_Config(&handlerUSART2);
	usart_Set_Priority(&handlerUSART2, e_USART_PRIORITY_6);


	// USER Button exti config
//	handlerUserButton.pGPIOx = GPIOC;
//	handlerUserButton.GPIO_PinConfig.GPIO_PinAltFunMode = AF0;
//	handlerUserButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
//	handlerUserButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
//	handlerUserButton.GPIO_PinConfig.GPIO_PinNumber = PIN_13;
//	handlerUserButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
//	handlerUserButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
//	handler_exti_userButon.edgeType = EXTERNAL_INTERRUPT_RISING_EDGE;
//	handler_exti_userButon.pGPIOHandler = &handlerUserButton;
//	handler_exti_userButon.priority     = e_EXTI_PRIORITY_6;
//	exti_Set_Priority(&handler_exti_userButon, e_EXTI_PRIORITY_6);
//	extInt_Config(&handler_exti_userButon);




}



void vTask_Menu( void * pvParameters )
{

   while(1){

	   //taskYIELD();
   }
}

void vTask_Print( void * pvParameters )
{

   while(1){

	   //taskYIELD();
   }
}

void vTask_Commands( void * pvParameters )
{

   while(1){

	   //taskYIELD();
   }
}
//void vTaskTwo( void * pvParameters )
//{
//
//   while(1){
////	   printf("%s\n",((char*) pvParameters));
//	   vTaskDelay((pdMS_TO_TICKS(10)));
//	   //GPIO_WritePin(&handlerPinA5, SET);
//	   //taskYIELD();
//   }
//}

void usart2Rx_Callback(void){

	rxData = getRxData();
//
//	BaseType_t xHigerPriorituTaskWoken;
//
//	xHigerPriorituTaskWoken = pdFALSE;

	//Verificamos que la cola aun no se encuentra llena
	xReturned = xQueueIsQueueFullFromISR(xQueue_InputData);
	// Si retorna que aun tiene espacio entoncesretorna falso

	if (xReturned != pdTRUE ){

		xQueueSendToBackFromISR(xQueue_InputData, (void*) &rxData,NULL);


	}else{


		if (rxData == '#'){

			xQueueReceiveFromISR(xQueue_InputData, (void *) &rxData, NULL);
		}

	}


}

void BasicTimer3_Callback(void){

	GPIOxTooglePin(&handlerPinA5);
}


//void callback_extInt13(void){
//
//	BaseType_t pxHiguerPriorityTaskWoken;
//	pxHiguerPriorityTaskWoken = pdFALSE;
//
//	//Notoficamos a la funcion del LED:
//	xTaskNotifyFromISR(xHandleTask_Led,0,eNoAction,&pxHiguerPriorityTaskWoken);
//
//
//}



void parseCommands(char *stringVector){

	sscanf(stringVector, "%s %u %u %u %s", cmd ,&firstParameter, &secondParameter, &thirdParameter, userMsg);

}
