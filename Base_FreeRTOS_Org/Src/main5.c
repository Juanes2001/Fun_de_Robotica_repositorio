/*
 * main5.c
 *
 *  Created on: Sep 15, 2024
 *      Author: juane
 */


//Incluir archivo punto h para el main
#include "main5.h"


//Prototipos
void process_command(command_t *cmd);
int extract_command(command_t *cmd);

//Mesajes para imprimir por la consola
const char *msg_invalid = "\n/// Ivalid option /// \n";
const char *msg_option_0 = "\n-----Selected option - 0 -----\n";
const char *msg_option_1 = "\n-----Selected option - 1 ------\n";
const char *msg_option_2 = "\n-----Selected option - 2 ------\n";
const char *msg_option_n = "\n-----Option out of range ------\n";



//---------------------------Inicio de definicion de funciones y variables base----------------------------------
GPIO_Handler_t handlerPinA5 = {0};    //Definimos un elemento del tipo GPIO_Handler_t (Struct) para el LED
void initSystem(void);                          //Definimos la cabecera para la configuracion
//---------------------------Fin de definicion de funciones y variables base----------------------------------

//-------------------------PIN MCO2--------------------------------
GPIO_Handler_t handler_GPIO_MCO2 = {0};       //Definimos un elemento del tipo GPIO_Handler_t (Struct) para utilizar el pin MCO2 con el fin de muestrear las frecuencias de los osciladores
void int_MCO2(void);                      //Funcion para la configuracion inicail del MCO2


//--------------------------USART-------------------------------
GPIO_Handler_t handlerPinTx = {0};       //Definimos un elemento del tipo GPIO_Handler_t (Struct) y USART_Handler_t para el uso del USB
GPIO_Handler_t handlerPinRx = {0};
USART_Handler_t handlerUSART = {0};
char rxData = 'w';                           //Variable que almacena el caracter leido

//-------------------------SEGGER-----------------------------------
//extern void SEGGER_UART_init(uint32_t);     //Le indicamos al sistema que hay una funcion para inicio de la comunicacion del SEGGER por UART

//-------------------------FreeRTOS-----------------------------------
//Definicion de variables para la configuracion inicial del FreeRTOS
//uint32_t SystemCoreClock = 100E6;
#define STACK_SIZE 200
//Variable para comprobar la creacion de la tarea
BaseType_t xReturned;

//Handler de las funciones de las Tareas
extern void vTask_Menu(void * pvParameters);
extern void vTask_Print(void * pvParameters);
extern void vTask_Commands(void * pvParameters);
//Handler de las Tareas
TaskHandle_t xHandleTask_Menu = NULL;
TaskHandle_t xHandleTask_Print = NULL;
TaskHandle_t xHandleTask_Commands = NULL;

//Handler de las Queue
QueueHandle_t xQueue_Print;
QueueHandle_t xQueue_InputData;

//Handler para el Software Timer
TimerHandle_t handler_led_timer;

//Variables adicionales aplicacion
state_t next_state = sMainMenu;


int main(void)
{
	RCC_enableMaxFrequencies(RCC_100MHz);
	//Configuracion inicial del sistema
	initSystem();
	//Activamos la unidad de punto flotante (FPU)
	SCB->CPACR    |= (0xF << 20);
	//Activamos del contador de Ticks
	DWT->CTRL    |= (1 << 0);


	//---------------------Inicio de uso de funciones para el funcionamiento del SEGGER----------------------
	//Necesaria para el SEGGER
//	vInitPrioGroupValue();
//	//Configuramos el puerto Serial para trabajar  con el SEGGER
//	SEGGER_UART_init(500000);
//	/* Primero configuramos */
//	SEGGER_SYSVIEW_Conf();
	//-----------------------Fin de uso de funciones para el funcionamiento del SEGGER----------------------


	//-----------------------Inicio cofiguracion de los elemntos del kernel de FreeRTOS----------------------

	//-------------------Configuracion Task--------------
	//Tarea Menu
	xReturned = xTaskCreate(
						vTask_Menu,       /* Function that implements the task. */
	                    "Task_Menu",          /* Text name for the task. */
	                    STACK_SIZE,      /* Stack size in words, not bytes. */
						NULL,    /* Parameter passed into the task. */
	                    2,/* Priority at which the task is created. */
	                    &xHandleTask_Menu);      /* Used to pass out the created task's handle. */
	configASSERT(xReturned == pdPASS);
	//Tarea Print
	xReturned = xTaskCreate(
						vTask_Print,       /* Function that implements the task. */
	                    "Task_Print",          /* Text name for the task. */
	                    STACK_SIZE,      /* Stack size in words, not bytes. */
						NULL,    /* Parameter passed into the task. */
	                    2,/* Priority at which the task is created. */
	                    &xHandleTask_Print);      /* Used to pass out the created task's handle. */
	configASSERT(xReturned == pdPASS);
	//Tarea comandos
	xReturned = xTaskCreate(
						vTask_Commands,       /* Function that implements the task. */
	                    "Task_Commands",          /* Text name for the task. */
	                    STACK_SIZE,      /* Stack size in words, not bytes. */
						NULL,    /* Parameter passed into the task. */
	                    2,/* Priority at which the task is created. */
	                    &xHandleTask_Commands);      /* Used to pass out the created task's handle. */
	configASSERT(xReturned == pdPASS);

	//-------------------Configuracion Queue--------------
	//Cola para recibir datos por consola
	xQueue_InputData = xQueueCreate(10, sizeof( char ) );
	configASSERT(xQueue_InputData != NULL);
	//cola para enviar datos por consola
	xQueue_Print = xQueueCreate(10, sizeof( size_t) );
	configASSERT(xQueue_Print != NULL);

	//-------------------Configuracion Timer--------------
	//Software Timer para el blink
	handler_led_timer = xTimerCreate("led_timer", pdMS_TO_TICKS(500), pdTRUE, 0, led_state_callback);
	xTimerStart(handler_led_timer, portMAX_DELAY);

	//-------------------Inicializacion Scheduler--------------
	//Inicia le Scheduler a funcionar
	vTaskStartScheduler();

	//-----------------------Fin cofiguracion de los elemntos del kernel de FreeRTOS----------------------

	//Si el scheduler se inicia correctamente no se ejecutada este while
	while(1)
	{
		__NOP();
	}
}


//------------------------------Inicio Configuracion del microcontrolador------------------------------------------
void initSystem(void)
{
	//---------------------------------Inicio de Configuracion GPIOx---------------------------------

	//---------------------------BlinkyLed--------------------------------
	//---------------PIN: PA5----------------
	//BLINKY LED

	handlerPinA5.pGPIOx = GPIOA;
	handlerPinA5.GPIO_PinConfig.GPIO_PinAltFunMode = AF0;
	handlerPinA5.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	handlerPinA5.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerPinA5.GPIO_PinConfig.GPIO_PinNumber = PIN_5;
	handlerPinA5.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinA5.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinA5);
	GPIO_WritePin(&handlerPinA5, SET);
	/*Opciones: GPIO_Tipo_x, donde x--->||IN, OUT, ALTFN, ANALOG ||| PUSHPULL, OPENDRAIN |||
	 * ||| LOW, MEDIUM, FAST, HIGH ||| NOTHING, PULLUP, PULLDOWN, RESERVED |||  AFx, 0-15 |||*/

	///////////////////////////////////////////Comunicación serial para comandos //////////////////////////////////////////////
			/////////A2 TX // A3 RX PARA USART 2 /////////
			////////A9 TX // A10 RX PARA USART 1 ////////

	//Comunicacion serial

	handlerPinTx.pGPIOx                             = GPIOA;
	handlerPinTx.GPIO_PinConfig.GPIO_PinAltFunMode  = AF7;
	handlerPinTx.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerPinTx.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerPinTx.GPIO_PinConfig.GPIO_PinNumber      = PIN_2;
	handlerPinTx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinTx.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_HIGH;
	GPIO_Config(&handlerPinTx);

	handlerPinRx.pGPIOx                             = GPIOA;
	handlerPinRx.GPIO_PinConfig.GPIO_PinAltFunMode  = AF7;
	handlerPinRx.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerPinRx.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerPinRx.GPIO_PinConfig.GPIO_PinNumber      = PIN_3;
	handlerPinRx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinRx.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_HIGH;
	GPIO_Config(&handlerPinRx);

	handlerUSART.ptrUSARTx                      = USART2;
	handlerUSART.USART_Config.USART_MCUvelocity = USART_50MHz_VELOCITY;
	handlerUSART.USART_Config.USART_baudrate    = USART_BAUDRATE_19200;
	handlerUSART.USART_Config.USART_enableInRx  = USART_INTERRUPT_RX_ENABLE;
	handlerUSART.USART_Config.USART_enableInTx  = USART_INTERRUPT_TX_DISABLE;
	handlerUSART.USART_Config.USART_mode        = USART_MODE_RXTX;
	handlerUSART.USART_Config.USART_parity      = USART_PARITY_NONE;
	handlerUSART.USART_Config.USART_stopbits    = USART_STOPBIT_1;
	handlerUSART.USART_Config.USART_datasize    = USART_DATASIZE_8BIT;
	USART_Config(&handlerUSART);
	usart_Set_Priority(&handlerUSART, e_USART_PRIORITY_6);


}

//------------------------------Fin Configuracion del microcontrolador------------------------------------------



//----------------------------Inicio de la definicion de las funciones ISR---------------------------------------


//-------------------------USARTRX--------------------------------
//Definimos la funcion que se desea ejecutar cuando se genera la interrupcion por el USART2
void usart2Rx_Callback(void)
{
	rxData = getRxData();

	//Se define variable para verificar si una tarea de mayor proridad esta lista para Running
	BaseType_t pxHigherPriorityTaskWoken;
	(void) pxHigherPriorityTaskWoken;
	pxHigherPriorityTaskWoken = pdFALSE;

	//Verificamos que la cola aun no se encuentra llena
	xReturned = xQueueIsQueueFullFromISR(xQueue_InputData);

	//Por tanto si es True entonces aun no hay espacio
	if(xReturned != pdTRUE)
	{
			//Se envia caracter a la cola
			xQueueSendToBackFromISR(xQueue_InputData, (void *)&rxData, NULL);
	}
	else
	{
		if(rxData == '#')
		{
			//Se recibe mensaje de la cola
			xQueueReceiveFromISR(xQueue_InputData, (void *)&rxData, NULL);
			//Se envia mensaje a la cola
			xQueueSendToBackFromISR(xQueue_InputData, (void *)&rxData, NULL);
		}
	}
	//Se envia notificacion al command Task
	if(rxData == '#')
	{
		xTaskNotifyFromISR(xHandleTask_Commands, 0, eNoAction, NULL);
	}
}

//----------------------------Fin de la definicion de las funciones ISR----------------------------------------


//----------------------Funciones referentes a cada tarea---------------------
//------Tarea Menu-------
void vTask_Menu(void * pvParameters)
{
	//Definimos variables
	uint32_t cmd_addr;
	command_t *cmd;
	int option;

	//Mensaje inicial del menu
	const char* msg_menu = "=======================\n"
			               "|         Menu        |\n"
						   "=======================\n"
						   "LED effect    ---> 0\n"
			               "Date and Time ---> 1\n"
			               "Exit          ---> 2\n"
			               "Enter your choice here : ";
	while(1)
	{
		//Se envia a imprimir el mensaje a la consola
		xQueueSend(xQueue_Print, &msg_menu, portMAX_DELAY);
		//Se espera por el comando a ejecutar
		xTaskNotifyWait(0,0,&cmd_addr, portMAX_DELAY);
		cmd = (command_t* ) cmd_addr;

		//Se verificamos si se tiene un solo caracter
		if(cmd->len == 1)
		{
			//Se transforma un ASCII a un numero 1...
			option = cmd->payload[0] - 48;

			switch(option){
			case 0:
				//Se envia la opcion especificada
				xQueueSend(xQueue_Print, &msg_option_0, portMAX_DELAY);
				//Notificacion del cambio de estado
				next_state = sMainMenu;
				xTaskNotify(xHandleTask_Menu, 0, eNoAction);

				break;
			case 1:
				//Se envia la opcion especificada
				xQueueSend(xQueue_Print, &msg_option_1, portMAX_DELAY);
				//Notificacion del cambio de estado
				next_state = sMainMenu;
				xTaskNotify(xHandleTask_Menu, 0, eNoAction);

				break;
			case 2:
				//Se envia la opcion especificada
				xQueueSend(xQueue_Print, &msg_option_2, portMAX_DELAY);
				//Notificacion del cambio de estado
				next_state = sMainMenu;
				xTaskNotify(xHandleTask_Menu, 0, eNoAction);

				break;
			default:
				//Se envia la opcion especificada
				xQueueSend(xQueue_Print, &msg_option_n, portMAX_DELAY);
				continue; //Se envia la instruccion al sistema de nuevo a esperar
			}
		}
		else
		{
			//Se envia la opcion especificada
			xQueueSend(xQueue_Print, &msg_invalid, portMAX_DELAY);
			//Notificacion del cambio de estado
			next_state = sMainMenu;
			xTaskNotify(xHandleTask_Menu, 0, eNoAction);
		}

		//La tarea se queda en un estado de espera por un tiempo indefinido
		xTaskNotifyWait(0,0,NULL, portMAX_DELAY);
	}

}

//-------Tarea de Imprimir-------
void vTask_Print(void * pvParameters)
{
	//Variable para guardad mensaje a enviar
	uint32_t *msg;

	while(1)
	{
		//Se espera por el puntero del mensaje
		xQueueReceive(xQueue_Print, &msg, portMAX_DELAY);
		//Enviamos por puerto serial dicho mensaje
		writeMsg(&handlerUSART, (char *) msg);
	}
}

//-------Tarea de los comando-------
void vTask_Commands(void * pvParameters)
{
	BaseType_t notify_status = {0};
	command_t cmd = {0};

	while(1)
	{
		//Se espera por la notificacion de la interrupcion
		notify_status = xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		//Si es verdadero se recibe una notificacion
		if(notify_status == pdTRUE)
		{
			//Se procesa el comando recibido
			process_command(&cmd);
		}
	}
}

//Funcion que ayuda en el procesamiento del comando
void process_command(command_t *cmd)
{
	extract_command(cmd);

	switch(next_state)
	{
	case sMainMenu:
		//Notificamos a la tarea respectiva
		xTaskNotify(xHandleTask_Menu, (uint32_t) cmd, eSetValueWithoutOverwrite);
		break;

	case sLedEffect:
		//xTaskNotify(xHandleTask_Leds, (uint32_t) cmd, eSetValueWithoutOverwrite);
		break;

	case sRtcMenu:
		//xTaskNotify(xHandleTask_Rtc, (uint32_t) cmd, eSetValueWithoutOverwrite);
		break;
	default:
		__NOP();
		break;
	}
}
//Funcion para obtener el comando
int extract_command(command_t *cmd)
{
	//Definicion de variables
	uint8_t item;
	uint8_t counter_j = 0;
	BaseType_t status;
	//Se verifica si hay un nuevo mensaje
	status = uxQueueMessagesWaiting(xQueue_InputData);
	if(status == 0)
	{
		return -1;
	}
	do{
		//Recibimos un elemento y lo montamos en item
		status = xQueueReceive(xQueue_InputData, &item, 0);
		if(status ==pdTRUE){
			//Vamos llenando el arreglo del comando
			cmd->payload[counter_j++] = item;
		}
	}while(item != '#');

	//Agregamos el elemento nulo y ademas definimos el largo del mensaje
	cmd->payload[counter_j - 1] = '\0';
	cmd->len = counter_j -1;

	return 0;
}




//----------------------Inicio de la definicion de las funciones del Software Timer----------------------------------
void led_state_callback(TimerHandle_t xTimer)
{
	//Cambio estado opuesto Led
	GPIOxTooglePin(&handlerPinA5);
}
