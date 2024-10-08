/*
 * main4.c
 *
 *  Created on: Aug 16, 2024
 *      Author: juan
 */


#include <stm32f4xx.h>

//LIBRERIAS FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

//LIBRERIAS DE C
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//LIBRERIAS DE PERIFERAL DRIVERS
#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "PwmDriver.h"
#include "EXTIDriver.h"
#include "RCCHunMHz.h"
#include "USARTxDriver.h"

/* tamaño del stack */
#define STACK_SIZE 200

/*definicion de variables del sistema */
uint32_t SystemCoreClock = 100000000;

/* Definición de Perifericos del nuestras librerias para controlar un LED (1)*/
GPIO_Handler_t handlerLedPin = {0};

/* Definición de Periphericos del nuestras librerias para recibir una interrupcion EXTI (2)*/
GPIO_Handler_t handlerUserButton = {0};
EXTI_Config_t handlerExtiButton = {0};

/* Definición de Periphericos del nuestras librerias para recibir el manejo del USART (3)*/
GPIO_Handler_t handlerPinRx = {0};
GPIO_Handler_t handlerPinTx = {0};
USART_Handler_t handlerUART = {0};

/* Definición de Variable auxiliar para la recepcion serial*/
uint8_t rxData = 0;

/* Elemento del freeRTOS con el que se recibe el resultado de la creación de
 * tareas, y otros elementos del kernel del freeRTOS (1)*/
BaseType_t xReturned;

/* Definicion de un semaforo binario y un semaforo contador (2)*/
SemaphoreHandle_t xBinarySemaphore;
SemaphoreHandle_t xCounterSemaphore;

/*  Definición de Software timer para el led de estado (1)*/
TimerHandle_t xTimerHandler;

TaskHandle_t xHandleTask_Counting = NULL;
TaskHandle_t xHandleTask_Print = NULL;

QueueHandle_t xQueue_Print;

const char *msg_working = "\n - - Working - - \n";
char auxMsg[64] = {0};
//char *dummy = auxMsg;

volatile uint8_t printOneTime = 0;

volatile uint16_t randomNumber = 0;

/* cabeceras de las funciones que representan las tareas que ejecuta el FreeRTOS (2)*/
void vTask_Counting( void * pvParameters );
void vTask_Print( void * pvParameters );

/* Cabeceras de otras funciones, como el initsystem, la función para generar el número random
 * y el callback para el led de estado (3)*/
void initSystem(void);
uint16_t getRandomNumber(void);
void led_Callback (TimerHandle_t xTimer);

/* Funcion principal. Aca es donde sucede todo!! */
int main(void)
{
	/*Inicializacion del sistema:
	 * - Encender la FPU
	 * - Activar el contador de Ticks para debuging
	 * - Configurar adecuadamente el reloj principal del equipo para 100MHz
	 * - Lllamar a la funcion que inicializa nuestros perifericos (initSystem)
	 * */
	SCB -> CPACR |= (0xF << 20);
	DWT -> CTRL    |= (1 << 0);
	initSystem();

	/* Crear las tareas que necesita el programa*/
	/* Creando la Tarea Counter */
	xReturned = xTaskCreate(vTask_Counting,
							"Task-contador",
							STACK_SIZE,
							NULL,
							1,
							&xHandleTask_Counting);


    configASSERT(xReturned == pdPASS);

    /* Creando la Tarea Print */
    xReturned = xTaskCreate (vTask_Print,
    						"Task-print",
							STACK_SIZE,
							NULL,
							3,
							&xHandleTask_Print);

    configASSERT(xReturned == pdPASS);

    /* Crear la cola para la tarea de impresion*/
    xQueue_Print = xQueueCreate(10, sizeof(size_t));
    configASSERT(xQueue_Print != NULL);  // verificamos que se ha creado la queue correctamente.

    /* Creando el semaforo binario (1)*/
   	xBinarySemaphore = xSemaphoreCreateBinary();
   	configASSERT(xBinarySemaphore != NULL);  // verificamos que se ha creado la queue correctamente.

   	/* Creando el semaforo counter de 10 elementos, que inicia en 0 (1)*/
   	xCounterSemaphore = xSemaphoreCreateCounting(10,0);
   	configASSERT(xCounterSemaphore != NULL);  // verificamos que se ha creado la queue correctamente.

    /* Creando el timer que controla el blinky del led de estado, con un periodo de 350ms (1)*/
   	xTimerHandler = xTimerCreate("ledTimer",
								pdMS_TO_TICKS(500),
								pdTRUE,
								(void*) 0,
								led_Callback);

   	/* Activar el Timer que controla el led (1)*/
   	xTimerStart(xTimerHandler, portMAX_DELAY);

    /* Start the created tasks running. */
    vTaskStartScheduler();


    /* Loop forever */
	while(1){
		/*Si llegamos aca, es que algo salio mal...*/
	}
}




void initSystem(void){

	//Habilitamos la maxima velocidad del micro
	RCC_enableMaxFrequencies();


	// Configuramos el pin del led de estado A5
	handlerLedPin.pGPIOx = GPIOA;
	handlerLedPin.GPIO_PinConfig.GPIO_PinAltFunMode = AF0;
	handlerLedPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	handlerLedPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerLedPin.GPIO_PinConfig.GPIO_PinNumber = PIN_5;
	handlerLedPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerLedPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerLedPin);
	GPIO_WritePin(&handlerLedPin, RESET);


	// configuramos el USART 2 pRA COMUNICACION SERIAL
	handlerPinRx.pGPIOx = GPIOA;
	handlerPinRx.GPIO_PinConfig.GPIO_PinAltFunMode = AF7;
	handlerPinRx.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	handlerPinRx.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerPinRx.GPIO_PinConfig.GPIO_PinNumber = PIN_3;
	handlerPinRx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinRx.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinRx);


	handlerPinTx.pGPIOx = GPIOA;
	handlerPinTx.GPIO_PinConfig.GPIO_PinAltFunMode = AF7;
	handlerPinTx.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	handlerPinTx.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerPinTx.GPIO_PinConfig.GPIO_PinNumber = PIN_2;
	handlerPinTx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinTx.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinTx);

	handlerUART.ptrUSARTx                      = USART2;
	handlerUART.USART_Config.USART_MCUvelocity = USART_50MHz_VELOCITY;
	handlerUART.USART_Config.USART_baudrate    = USART_BAUDRATE_19200;
	handlerUART.USART_Config.USART_enableInRx  = USART_INTERRUPT_RX_ENABLE;
	handlerUART.USART_Config.USART_enableInTx  = USART_INTERRUPT_TX_DISABLE;
	handlerUART.USART_Config.USART_mode        = USART_MODE_RXTX;
	handlerUART.USART_Config.USART_parity      = USART_PARITY_NONE;
	handlerUART.USART_Config.USART_stopbits    = USART_STOPBIT_1;
	handlerUART.USART_Config.USART_datasize    = USART_DATASIZE_8BIT;
	USART_Config(&handlerUART);
	usart_Set_Priority(&handlerUART, e_USART_PRIORITY_6);



	// configuracion del Exti para interrupcion del Usser Button
	handlerUserButton.pGPIOx = GPIOC;
	handlerUserButton.GPIO_PinConfig.GPIO_PinAltFunMode = AF0;
	handlerUserButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	handlerUserButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerUserButton.GPIO_PinConfig.GPIO_PinNumber = PIN_13;
	handlerUserButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerUserButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	handlerExtiButton.edgeType = EXTERNAL_INTERRUPT_RISING_EDGE;
	handlerExtiButton.pGPIOHandler = &handlerUserButton;
	handlerExtiButton.priority     = e_EXTI_PRIORITY_6;
	exti_Set_Priority(&handlerExtiButton, e_EXTI_PRIORITY_6);
	extInt_Config(&handlerExtiButton);


}


/* Funcion que gobierna a la tarea que muestra como funciona un semaforo contador */
void vTask_Counting( void * pvParameters )
{

	const TickType_t xMaxExpectedBlockTime = pdMS_TO_TICKS( 1000 );

    while(1)
    {
    	/* En este if() el sistema espera que se tenga un elemento disponible en el semaforo
    	 * cuando esta disponible actual -> imprime un mensaje que dice "working"*/
    	if ( xSemaphoreTake(xCounterSemaphore,xMaxExpectedBlockTime) == pdPASS) {

    		/* Este bloque es para imprimir solo 1 vez el valor del numero aleatorio*/
    		if(printOneTime == 1){
    			sprintf(auxMsg, "counter i = %d\n", randomNumber);
    			char *dummy = auxMsg;
    			xQueueSend(xQueue_Print, &dummy, portMAX_DELAY);
    			printOneTime = 0;
    		}
    		/* Imprime las peticiones de trabajo que genera el numero aleatorio en la interupcion*/
    		writeMsg(&handlerUART,(char*) msg_working);
    	}
    }
}

/* Funcion que gobierna a la tarea Print */
void vTask_Print( void * pvParameters )
{

	uint32_t *msg;
    while(1)
    {
        /* Task code goes here. */
    	xQueueReceive(xQueue_Print, &msg, portMAX_DELAY);
    	//Escribimos el mensaje en la terminal
    	writeMsg(&handlerUART, (char*) msg);

    }
}

/*
 * Generar un numero aleatorio entre 1 y 10
 * la funcion sran(number), es para generar una semilla, de forma
 * que rand() genere un numero diferente siempre.
 * La funcion srand(number) requiere siempre un numero diferente, para
 * generar siempre un numero diferente...
 * */
uint16_t getRandomNumber(void)
{
	srand(xTaskGetTickCount());
	int randomnumber;
	randomnumber = (rand() % 10) +1;
	return (uint16_t)randomnumber;
}

/*Controla el estado del Led */
void led_Callback( TimerHandle_t xTimer )
{
	/* Toogle LED */
	GPIOxTooglePin(&handlerLedPin);
}

/* Interrupcion lanzada por el encider del BlueMotor. */
void callback_extInt13(void)
{
	/* Activar la variable printOneTime*/
	printOneTime = SET;
	/* Generar un numero aleatorio */
	randomNumber = getRandomNumber();

	BaseType_t xHigherPriorityTaskWoken;
	(void) xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	//(void)pxHigherPriorityTaskWoken;

	/* Hacer un for que se ejecute tantas veces como el numero aleatorio lo indica
	 * y allí cargar el semaforo (ejemplo binario y ejemplo counter)*/
   	for (uint16_t ii = 0; ii < randomNumber; ii++) {
   		/*cargar el semaforo e indicar que hay un cambio en las funciones.*/
   		xSemaphoreGiveFromISR(xCounterSemaphore, &xHigherPriorityTaskWoken);
	}
}

/* Interrupcion debida al puerto serial */
void usart2Rx_Callback(void)
{
	/* Recibir un dato en el serial y no hacer nada mas*/
	rxData = getRxData();
}
