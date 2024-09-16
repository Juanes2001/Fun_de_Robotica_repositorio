/*
 * main2.c
 *
 *  Created on: Aug 2, 2024
 *      Author: juan
 */

 // FUNCIONO LOS CAMBIOS AHORA SI FUNCIONO
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
#include "MPUAccel.h"
#include "I2CDriver.h"
#include "MotorsDriver.h"
#include "PosRobt.h"
#include "DMA.h"



// Definicion de algunas estructuras, con estas podemos almacenar el estado que queremos entrar dependiendo del comando que se seleccione

typedef struct{
	uint8_t payload[64];
	int functionType;
}command_t;

// En esta estructura enumerada se tendran todos los estados posibles, donde mediante comandos nos iremos a un estado u otro
typedef enum {
	sMainMenu = 0,
	sGo,
	sGoTo,
	sStop,
	sRoll,
	sRollTo,
	sAstar,
	sSquare
}state_t;


/*definicion de variables del sistema*/

// Aqui designamos al heap cada tarea cuanta memoria en words almacenará
#define STACK_SIZE 200

// Velocidad del MCU
uint32_t SystemCoreClock = 100000000;

// Definicion de la variable xReturned donde se almacenara el estado de todas las tareas creadas
BaseType_t xReturned;
/*Cabecera de la funcion de las tareas */
void vTask_Menu( void * pvParameters );
void vTask_Print( void * pvParameters );
void vTask_Commands( void * pvParameters );
void vTask_Stop( void * pvParameters );
void vTask_Go( void * pvParameters );
void vTask_Control( void * pvParameters );
void vTask_GoTo( void * pvParameters );

//Cabecera de la funcion Timer de FreeRTOS
void led_state_callback (TimerHandle_t xTimer);

//Tareas
TaskHandle_t xHandleTask_Menu     = NULL; // Handler de la tarea de Menu
TaskHandle_t xHandleTask_Print    = NULL; // Handler de la tarea dE PRINT
TaskHandle_t xHandleTask_Commands = NULL; // Handler de la tarea de Comandos
TaskHandle_t xHandleTask_Stop     = NULL; // Handler de la tarea de Stop
TaskHandle_t xHandleTask_Go       = NULL; // Handler de la tarea de Go
TaskHandle_t xHandleTask_Control  = NULL; // Handler de la tarea de PID
TaskHandle_t xHandleTask_GoTo     = NULL; // Handler de la tarea de Ir a

//Colas
QueueHandle_t xQueue_Print;
QueueHandle_t xQueue_InputData;

//Timer FreeRTOS
TimerHandle_t handler_led_timer;


void process_command (command_t *cmd);
int extract_command (command_t *cmd);

// CALIBRACIÓN ANGULO
float calibracionGyros (MPUAccel_Config *ptrMPUAccel, uint8_t axis);

//Algunos mensajes String necesarios para la comunicacion
const char *msg_invalid = "\n ////Invalid option /////\n";
const char *msg_option_1= "\n------ Selected Option - sGo ------ \n";
const char *msg_option_2= "\n----- Selected Option - sGoTo ----- \n";
const char *msg_option_3= "\n------ Selected Option - sRoll ---- \n";
const char *msg_option_4= "\n----Selected Option - sRollTo ----- \n";
const char *msg_no_smphr= "\n-------Semaphore no activated------ \n";


//Definición Handlers
//GPIO
//Pin del User blinky
GPIO_Handler_t handlerPinA5         = {0};

//Pin de Reset
GPIO_Handler_t handlerUserButton    = {0};


//Pines PWM que controlan la velocidad de los motores
GPIO_Handler_t handlerPinPwm_1      = {0};
GPIO_Handler_t handlerPinPwm_2      = {0};

//Pines de comunicacion USART
GPIO_Handler_t handlerPinRx         = {0};
GPIO_Handler_t handlerPinTx         = {0};

//Pin para visualizar la velocidad del micro
GPIO_Handler_t handlerMCO2Show      = {0};

//Pines para encendido y apagado de los motores
GPIO_Handler_t handlerEn2PinC11     = {0};
GPIO_Handler_t handlerEn1PinC10     = {0};

//Pines de salida para la direccion de las ruedas
GPIO_Handler_t handlerIn2PinD2      = {0};
GPIO_Handler_t handlerIn1PinC12     = {0};

//Pines para lectura de encoders
GPIO_Handler_t handlerEncoder1PinC1 = {0};
GPIO_Handler_t handlerEncoder2PinC3 = {0};

// Pines para I2C1
GPIO_Handler_t handler_PINB8_I2C1   = {0};
GPIO_Handler_t handler_PINB9_I2C1   = {0};

//Extis
EXTI_Config_t handlerExtiConEnc_1 = {0};
EXTI_Config_t handlerExtiConEnc_2 = {0};

//Timers
BasicTimer_Handler_t handlerTimerBlinky = {0}; // Timer 3
BasicTimer_Handler_t handlerTIM2_PARAMETROS_MOVIMIENTO    = {0}; // Timer 2
BasicTimer_Handler_t handlerTIM4_time   = {0}; // Timer 4

//PWMs
PWM_Handler_t handlerPWM_1 = {0}; // Timer 5
PWM_Handler_t handlerPWM_2 = {0}; // Timer 5

//Usart
USART_Handler_t handlerUSART = {0};


//I2C
I2C_Handler_t handler_I2C1 = {0};

//DMA
DMA_Handler_t *handler_DMA1[2];

//MPUAccel
MPUAccel_Config handler_MPUAccel_6050 ={0};

// Motor Drivers
Motor_Handler_t *handler_Motor_Array[2]; // Handler para cada motor, 0--> izquierdo; 1--> derecho
Motor_Handler_t handlerMotor1_t;
Motor_Handler_t handlerMotor2_t;

////////ESTRUCTURAS
// Estructura de estados
typedef enum{
	Line = 0,
	Roll
}Op_Mode_t;

typedef struct{
	Op_Mode_t Mode;
	uint8_t direction_s_r;
}state_dir_t;

//DEFINICION DE FUNCIONES

void inSystem (void);
void resetParameters(void);
float calibracionGyros (MPUAccel_Config *ptrMPUAccel, uint8_t axis);
void getAngle(MPUAccel_Config *ptrMPUAccel,float angle_init, double calibr,Parameters_Position_t *ptrParameter_position);
void On_motor_Straigh_Roll(Motor_Handler_t *ptrMotorhandler[2], state_dir_t operation_mode_dir);
void get_measuremets_parameters(Motor_Handler_t *ptrMotorHandler[2], Parameters_Position_t *ptrParameter_position, state_dir_t operation_mode_dir);
void change_dir_straigh_Roll(Motor_Handler_t *ptrMotorhandler[2], state_dir_t operation_mode_dir);
void set_direction_straigh_roll (Motor_Handler_t *ptrMotorhandler[2], state_dir_t operation_mode_dir);
void stop (Motor_Handler_t *ptrMotorhandler[2]);
void int_Config_Motor(Motor_Handler_t *ptrMotorhandler[2],
					  Parameters_Position_t *ptrPosHandler,
					  Parameters_Path_t *ptrPathHandler ,
					  PID_Parameters_t *ptrPIDHandler);
void PID_calc(PID_Parameters_t *ptrPIDHandler, float time_of_sampling, float setpoint, float current_measure);
void fillComand(void);
void PID_control(Motor_Handler_t *ptrMotorhandler[2],
				 Parameters_Path_t *ptrPathHandler,
				 Parameters_Position_t *ptrPosHandler,
				 PID_Parameters_t *ptrPIDHandler);

void go(Motor_Handler_t *ptrMotorhandler[2],
		MPUAccel_Config *ptrMPUhandler,
		Parameters_Position_t *ptrPosHandler ,
		Parameters_Path_t *ptrPathHandler,
		PID_Parameters_t *ptrPIDHandler,
		double calib ,
		uint8_t *fAnglulo,
		uint8_t *fMeasurements,
		uint8_t *fcontrol,
		char buff[64],
		state_dir_t operation_mode_dir);

int goTo(Motor_Handler_t *ptrMotorhandler[2],
		 MPUAccel_Config *ptrMPUhandler,
		 Parameters_Position_t *ptrPosHandler ,
		 Parameters_Path_t *ptrPathHandler,
		 PID_Parameters_t *ptrPIDHandler,
		 double calib ,
		 uint8_t *fAnglulo,
		 uint8_t *fMeasurements,
		 uint8_t *fcontrol,
		 char buff[64],
		 uint32_t distance_mm ,
		 state_dir_t operation_mode_dir);

void roll(Motor_Handler_t *ptrMotorhandler[2],
		MPUAccel_Config *ptrMPUhandler,
		Parameters_Position_t *ptrPosHandler ,
		Parameters_Path_t *ptrPathHandler,
		double calib ,
		uint8_t *fAnglulo,
		uint8_t *fMeasurements,
		uint8_t *fcontrol,
		char buff[64],
		state_dir_t operation_mode_dir);

int extract_info ( command_t *cmd ,
				   char data[64],
				   unsigned char firstParameter[10],
				   unsigned char secondParameter[10],
				   unsigned char thirdParameter[10],
				   unsigned int *fparam,
				   unsigned int *sparam,
				   unsigned int *tparam);

//-----Macros------
#define distanceBetweenWheels 10600             //Distacia entre ruedas     106,00 [mm]
#define D1 5170                                 //Diametro rueda izquierda  51,70  [mm]
#define D2 5145                                 //Diametro rueda Derecha    51,45 [mm]
#define Ce 72                                   //Numero de interrupciones en el incoder

// Variables para los comandos
char bufferReception[64] = {0};
uint8_t counterReception = 0;
uint8_t doneTransaction = RESET;
uint8_t rxData = '\0';
//char cmd[32];
unsigned char firstParameter[10] = {0};
unsigned char secondParameter[10] = {0};
unsigned char thirdParameter[10]  = {0};

unsigned int fparam = 0;
unsigned int sparam = 0;
unsigned int tparam = 0;

char data[64];
char userMsg[64];

//////Banderas y estados-----------
state_dir_t Mode_dir      = {0};
Op_Mode_t Mode              = Line;
state_t next_state = sMainMenu;
uint8_t flag_angulo       = 0;
uint8_t flag_measurements = 0;
uint8_t flag_Go_Straigh   = 0;
uint8_t flag_GoTo_Straigh = 0;
uint8_t flag_control      = 0;
uint8_t flag_Roll         = 0;
uint8_t flag_RollTo         = 0;
uint8_t Done = 0;
uint8_t wrong_command = 0;
uint8_t end = 0;


// TIEMPOS DE SAMPLEO Y CONTEO PARA DEFINICION DE PARAMETROS
uint8_t timeAction_TIMER_Sampling = 13;            // Cantidad de cuentas para
uint16_t time_accumulated = 0;
uint16_t counting_action = 0;                     //Contador para la accion
uint32_t time_accion = 0;

//-----Variables PID-----------
PID_Parameters_t parameter_PID_distace = {0};        //estructura para los parametros del PID

/// Variables para Odometria
Parameters_Path_t parameters_Path_Robot    = {0};           //Estructura que almacena los parametros del camino a recorrer
Parameters_Position_t parameters_Pos_Robot = {0}; 	//Estructura que almacena la posicion del robot

double cal_Gyro = 0;   // variable que almacena la calibración del giroscopio
float ang_d = 0;
float sum_ang = 0;
float promAng = 0;
float paso_mm_1 = ((M_PI*D1)/(100*Ce)); // Numero de milimetros por paso de la rueda izquierda [mm]
float paso_mm_2 = ((M_PI*D2)/(100*Ce)); // Numero de milimetros por paso de la rueda derecha   [mm]
double ang_for_Displament = 0;
double ang_complementary = 0;
float velSetPoint = 0;
int counter;
float vel_Setpoint_1;
float vel_Setpoint_2;
double distance_to_go = 0;

// VARIABLES VARIAS DEL ROBOT
#define fixed_dutty 28 // Fixed dutty cycle, velocidad constante
#define fixed_sample_period 16 // Periodo en milisegundos de muestreo de datos de encoder

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

   	RCC_enableMaxFrequencies(RCC_100MHz);
	inSystem ();

	cal_Gyro = calibracionGyros(&handler_MPUAccel_6050, CALIB_Z);


	/////////////////////////////////TAREA DEL MENU//////////////////////////////////////

	xReturned = xTaskCreate(
					vTask_Menu,             // Nombre de la funcion de la tarea relacionada
					"Task-MENU",            // Nombre con el que se puede definir la tarea
					STACK_SIZE,             // Tamaño de la tarea en el heap
					NULL,                   // parametro pasado a la tarea
					3,                      // Prioridad de la tarea
					&xHandleTask_Menu );    // Handler de la tarea respectiva


	 configASSERT( xReturned == pdPASS ); // Nos aseguramos de que se creo la tarea de una forma correcta


	/////////////////////////////////TAREA DE IMPRESIÓN//////////////////////////////////////

	xReturned = xTaskCreate(vTask_Print, "Task-Print",STACK_SIZE,NULL,3,&xHandleTask_Print );

	 configASSERT( xReturned == pdPASS ); // Nos aseguramos de que se creo la tarea de una forma correcta

	 /////////////////////////////////TAREA DE COMANDOS //////////////////////////////////////

	xReturned = xTaskCreate(vTask_Commands,"Task-Commands",STACK_SIZE,NULL,3,&xHandleTask_Commands );

	 configASSERT( xReturned == pdPASS );// Nos aseguramos de que se creo la tarea de una forma correcta

	 /////////////////////////////////TAREA DE STOP //////////////////////////////////////


	xReturned = xTaskCreate(vTask_Stop,"Task-Stop",STACK_SIZE,NULL,3,&xHandleTask_Stop );

	 configASSERT( xReturned == pdPASS );// Nos aseguramos de que se creo la tarea de una forma correcta

	 /////////////////////////////////TAREA DE GO//////////////////////////////////////

	xReturned = xTaskCreate(vTask_Go,"Task-Go",STACK_SIZE,NULL,3,&xHandleTask_Go );

	 configASSERT( xReturned == pdPASS );// Nos aseguramos de que se creo la tarea de una forma correcta

	 /////////////////////////////////TAREA DE PID//////////////////////////////////////

	xReturned = xTaskCreate(vTask_Control,"Task-Control",STACK_SIZE,NULL,3,&xHandleTask_Control );

	 configASSERT( xReturned == pdPASS );// Nos aseguramos de que se creo la tarea de una forma correcta

	 /////////////////////////////////TAREA DE IR A//////////////////////////////////////

	xReturned = xTaskCreate(vTask_GoTo,"Task-GoTo",STACK_SIZE,NULL,3,&xHandleTask_GoTo );

	 configASSERT( xReturned == pdPASS );// Nos aseguramos de que se creo la tarea de una forma correcta



	 //Creacion de colas
	 // Para cada funcion de crear se tiene que definir el largo de la cola,, y el
	 // largo de cada elemento de la cola.
	 xQueue_InputData = xQueueCreate(20,sizeof(char));
	 configASSERT(xQueue_InputData != NULL);// Verificamos que se creo la cola correctamente

	 //XQueue_Print = xQueueCreate (10, sizeof (struct AMessage *))
	 xQueue_Print = xQueueCreate(10,sizeof(size_t));
	 configASSERT(xQueue_Print != NULL); // Verificamos que se creo la cola correctamente

	 //Creando el timer de FreeRTOS


	 /* Start the created tasks running. */

	 handler_led_timer = xTimerCreate("led_timer",
			 	 	 	 	 	 	 pdMS_TO_TICKS(500),
									 pdTRUE,
									 (void *) 1,
									 led_state_callback);

	 xTimerStart(handler_led_timer, portMAX_DELAY);

	 // Definicion del semaforo para saltar interrupciiones y definir tareas de diferentes prioridades



	 vTaskStartScheduler();


    /* Loop forever */
	while(1){
		//Si se llega hasta aca es porque algo salio mal

	}
}


void inSystem (void){

	//Config del pin A8 salida de la velocidad del micro

//	handlerMCO2Show.pGPIOx                             = GPIOC;
//	handlerMCO2Show.GPIO_PinConfig.GPIO_PinNumber      = PIN_9 ;
//	handlerMCO2Show.GPIO_PinConfig.GPIO_PinAltFunMode  = AF0;
//	handlerMCO2Show.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
//	GPIO_Config(&handlerMCO2Show);

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


	// DEFINICION DEL TIM4 PARA DELAY
	inTIM4();


	//////////////////////////// INICIALIZAMOS EL ROBOT//////////////////////
	int_Config_Motor(handler_Motor_Array, &parameters_Pos_Robot, &parameters_Path_Robot, &parameter_PID_distace);


	//////////////////////////////////////////////////// Velocidad de motores //////////////////////////////////////////////


	//PWM
	// PWM motor 1
	handlerPinPwm_1.pGPIOx                             = GPIOA;
	handlerPinPwm_1.GPIO_PinConfig.GPIO_PinAltFunMode  = AF2;
	handlerPinPwm_1.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerPinPwm_1.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerPinPwm_1.GPIO_PinConfig.GPIO_PinNumber      = PIN_0;
	handlerPinPwm_1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinPwm_1.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinPwm_1);

	handlerPWM_1.ptrTIMx            = TIM5;
	handlerPWM_1.config.channel     = PWM_CHANNEL_1;
	handlerPWM_1.config.duttyCicle  = fixed_dutty;
//	counter = fixed_dutty;
	handlerPWM_1.config.periodo     = 33; // se maneja 25 hz por testeo
	handlerPWM_1.config.prescaler   = PWM_SPEED_100MHz_1us;
	handlerPWM_1.config.polarity    = PWM_ENABLE_POLARITY;
	handlerPWM_1.config.optocoupler = PWM_ENABLE_OPTOCOUPLER;
	pwm_Config(&handlerPWM_1);
	startPwmSignal(&handlerPWM_1);

	//PWM motor 2
	handlerPinPwm_2.pGPIOx                             = GPIOA;
	handlerPinPwm_2.GPIO_PinConfig.GPIO_PinAltFunMode  = AF2;
	handlerPinPwm_2.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerPinPwm_2.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerPinPwm_2.GPIO_PinConfig.GPIO_PinNumber      = PIN_1;
	handlerPinPwm_2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinPwm_2.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinPwm_2);

	handlerPWM_2.ptrTIMx            = TIM5;
	handlerPWM_2.config.channel     = PWM_CHANNEL_2;
	handlerPWM_2.config.duttyCicle  = fixed_dutty;
	handlerPWM_2.config.periodo     = 33;// se maneja 25 hz por testeo
	handlerPWM_2.config.prescaler   = PWM_SPEED_100MHz_1us;
	handlerPWM_2.config.polarity    = PWM_ENABLE_POLARITY;
	handlerPWM_2.config.optocoupler = PWM_ENABLE_OPTOCOUPLER;
	pwm_Config(&handlerPWM_2);
	startPwmSignal(&handlerPWM_2);


	//////////////////////////////////////////////////// /////////////////// //////////////////////////////////////////////

	////////////////////////////////////// Enable 1 y 2, encendido y apagado de motores //////////////////////////////////////////////



	handlerEn1PinC10.pGPIOx                             = GPIOC;
	handlerEn1PinC10.GPIO_PinConfig.GPIO_PinAltFunMode  = AF0;
	handlerEn1PinC10.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
	handlerEn1PinC10.GPIO_PinConfig.GPIO_PinNumber      = PIN_10;
	handlerEn1PinC10.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerEn1PinC10.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerEn1PinC10.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerEn1PinC10);
	GPIO_WritePin_Afopt(&handlerEn1PinC10, RESET);

	handlerEn2PinC11.pGPIOx                             = GPIOC;
	handlerEn2PinC11.GPIO_PinConfig.GPIO_PinAltFunMode  = AF0;
	handlerEn2PinC11.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
	handlerEn2PinC11.GPIO_PinConfig.GPIO_PinNumber      = PIN_11;
	handlerEn2PinC11.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerEn2PinC11.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerEn2PinC11.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerEn2PinC11);
	GPIO_WritePin_Afopt(&handlerEn2PinC11, RESET);


	//////////////////////////////////////////////////// /////////////////// //////////////////////////////////////////////

	////////////////////////////////////// In 1 y 2, direccion de colores CW y CCW //////////////////////////////////////////////

	handlerIn1PinC12.pGPIOx                             = GPIOC;
	handlerIn1PinC12.GPIO_PinConfig.GPIO_PinAltFunMode  = AF0;
	handlerIn1PinC12.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
	handlerIn1PinC12.GPIO_PinConfig.GPIO_PinNumber      = PIN_12;
	handlerIn1PinC12.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerIn1PinC12.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerIn1PinC12.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerIn1PinC12);
	GPIO_WritePin_Afopt(&handlerIn1PinC12, RESET); // default SET
	handler_Motor_Array[0]->configMotor.dir = SET;

	handlerIn2PinD2.pGPIOx                             = GPIOD;
	handlerIn2PinD2.GPIO_PinConfig.GPIO_PinAltFunMode  = AF0;
	handlerIn2PinD2.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
	handlerIn2PinD2.GPIO_PinConfig.GPIO_PinNumber      = PIN_2;
	handlerIn2PinD2.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerIn2PinD2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerIn2PinD2.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerIn2PinD2);
	GPIO_WritePin_Afopt(&handlerIn2PinD2, RESET); // default SET
	handler_Motor_Array[1]->configMotor.dir = SET;

	//////////////////////////////////////////////////// /////////////////// //////////////////////////////////////////////

	////////////////////////////////////// Conteo de encoders motor 1 y motor 2//////////////////////////////////////////////


	handlerEncoder1PinC1.pGPIOx                             = GPIOC;
	handlerEncoder1PinC1.GPIO_PinConfig.GPIO_PinAltFunMode  = AF0;
	handlerEncoder1PinC1.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_IN;
	handlerEncoder1PinC1.GPIO_PinConfig.GPIO_PinNumber      = PIN_1;
	handlerEncoder1PinC1.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerEncoder1PinC1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerEncoder1PinC1.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	handlerExtiConEnc_1.pGPIOHandler                        = &handlerEncoder1PinC1;
	handlerExtiConEnc_1.edgeType                            = EXTERNAL_INTERRUPT_RASINGANDFALLING_EDGE;
	extInt_Config(&handlerExtiConEnc_1);
	exti_Set_Priority(&handlerExtiConEnc_1, e_EXTI_PRIORITY_6);

	handlerEncoder2PinC3.pGPIOx                             = GPIOC;
	handlerEncoder2PinC3.GPIO_PinConfig.GPIO_PinAltFunMode  = AF0;
	handlerEncoder2PinC3.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_IN;
	handlerEncoder2PinC3.GPIO_PinConfig.GPIO_PinNumber      = PIN_3;
	handlerEncoder2PinC3.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerEncoder2PinC3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerEncoder2PinC3.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	handlerExtiConEnc_2.pGPIOHandler                        = &handlerEncoder2PinC3;
	handlerExtiConEnc_2.edgeType                            = EXTERNAL_INTERRUPT_RASINGANDFALLING_EDGE;
	extInt_Config(&handlerExtiConEnc_2);
	exti_Set_Priority(&handlerExtiConEnc_2, e_EXTI_PRIORITY_6);


	//////////////////////////////////////////////////// /////////////////// //////////////////////////////////////////////

	///////////////////////////////////////////Comunicación serial para comandos //////////////////////////////////////////////
		/////////A2 TX // A3 RX PARA USART 2 /////////
		////////A9 TX // A10 RX PARA USART 1 ////////

	//Comunicacion serial

	handlerPinTx.pGPIOx                             = GPIOA;
	handlerPinTx.GPIO_PinConfig.GPIO_PinAltFunMode  = AF7;
	handlerPinTx.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerPinTx.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerPinTx.GPIO_PinConfig.GPIO_PinNumber      = PIN_9;
	handlerPinTx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinTx.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_HIGH;
	GPIO_Config(&handlerPinTx);

	handlerPinRx.pGPIOx                             = GPIOA;
	handlerPinRx.GPIO_PinConfig.GPIO_PinAltFunMode  = AF7;
	handlerPinRx.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerPinRx.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerPinRx.GPIO_PinConfig.GPIO_PinNumber      = PIN_10;
	handlerPinRx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinRx.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_HIGH;
	GPIO_Config(&handlerPinRx);

	handlerUSART.ptrUSARTx                      = USART1;
	handlerUSART.USART_Config.USART_MCUvelocity = USART_100MHz_VELOCITY;
	handlerUSART.USART_Config.USART_baudrate    = USART_BAUDRATE_19200;
	handlerUSART.USART_Config.USART_enableInRx  = USART_INTERRUPT_RX_ENABLE;
	handlerUSART.USART_Config.USART_enableInTx  = USART_INTERRUPT_TX_DISABLE;
	handlerUSART.USART_Config.USART_mode        = USART_MODE_RXTX;
	handlerUSART.USART_Config.USART_parity      = USART_PARITY_NONE;
	handlerUSART.USART_Config.USART_stopbits    = USART_STOPBIT_1;
	handlerUSART.USART_Config.USART_datasize    = USART_DATASIZE_8BIT;
	USART_Config(&handlerUSART);
	usart_Set_Priority(&handlerUSART, e_USART_PRIORITY_6);

	//////////////////////////////////////////////////// /////////////////// //////////////////////////////////////////////

	///////////////////////////////////////////Timer para el control de la velocidad//////////////////////////////////////////////

	handlerTIM2_PARAMETROS_MOVIMIENTO.ptrTIMx                           = TIM2;
	handlerTIM2_PARAMETROS_MOVIMIENTO.TIMx_Config.TIMx_interruptEnable  = BTIMER_ENABLE_INTERRUPT;
	handlerTIM2_PARAMETROS_MOVIMIENTO.TIMx_Config.TIMx_mode             = BTIMER_MODE_UP;
	handlerTIM2_PARAMETROS_MOVIMIENTO.TIMx_Config.TIMx_speed            = BTIMER_SPEED_100MHz_10us;
	handlerTIM2_PARAMETROS_MOVIMIENTO.TIMx_Config.TIMx_period           = fixed_sample_period;
	BasicTimer_Config(&handlerTIM2_PARAMETROS_MOVIMIENTO);
	TIM_SetPriority(&handlerTIM2_PARAMETROS_MOVIMIENTO, e_TIM_PRIORITY_6);


	//////////////////////////////////////////////////// /////////////////// //////////////////////////////////////////////

		////////////////////////////////Configuracion PINES B8 (SCL) B9 (SDA) e I2C1 //////////////////////////////////////////////


//	handler_DMA1[0]->ptrDMAType = DMA1;
//	handler_DMA1[0]->ptrDMAStream = DMA1_Stream0;
//
//	handler_DMA1[1]->ptrDMAType = DMA1;
//	handler_DMA1[1]->ptrDMAStream = DMA1_Stream6;
//	config_DMA(handler_DMA1);

	handler_PINB8_I2C1.pGPIOx                             = GPIOB;
	handler_PINB8_I2C1.GPIO_PinConfig.GPIO_PinAltFunMode  = AF4;
	handler_PINB8_I2C1.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handler_PINB8_I2C1.GPIO_PinConfig.GPIO_PinNumber      = PIN_8;
	handler_PINB8_I2C1.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_OPENDRAIN;
	handler_PINB8_I2C1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handler_PINB8_I2C1.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;

	handler_PINB9_I2C1.pGPIOx                             = GPIOB;
	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinAltFunMode  = AF4;
	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinNumber      = PIN_9;
	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_OPENDRAIN;
	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;

	handler_I2C1.ptrI2Cx = I2C1;
	handler_I2C1.I2C_Config.clkSpeed = MAIN_CLOCK_50_MHz_FOR_I2C;
	handler_I2C1.I2C_Config.slaveAddress = ADDRESS_DOWN;
	handler_I2C1.I2C_Config.modeI2C = I2C_MODE_FM;

	handler_MPUAccel_6050.ptrGPIOhandlerSCL  = &handler_PINB8_I2C1;
	handler_MPUAccel_6050.ptrGPIOhandlerSDA  = &handler_PINB9_I2C1;
	handler_MPUAccel_6050.ptrI2Chandler   = &handler_I2C1;
	handler_MPUAccel_6050.fullScaleACCEL  = ACCEL_2G;
	handler_MPUAccel_6050.fullScaleGYRO   = GYRO_250;
	configMPUAccel(&handler_MPUAccel_6050);

}


//////////////////////////////////////////////////////////////////////// MENU STATE/////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void vTask_Menu( void * pvParameters ){

	uint32_t cmd_addr;
	command_t *cmd;

	const char* msg_menu = "\n===============================================\n"
						   "|                     MENU                    |\n"
						   "sGo #dir ---------> 1--> Adelante , 0--> Atras \n"
						   "sGoTo #dir #distance [mm]                      \n"
						   "Enter your choice here:";

	while (1){

		// Envia a imprimir en la consola lo que se debe mostrar en el menu
		xQueueSend(xQueue_Print, &msg_menu, portMAX_DELAY);

		// Se queda esperando a recibir el comando que se debe ejecutar
		xTaskNotifyWait (0,0,&cmd_addr, portMAX_DELAY);
		cmd = (command_t *) cmd_addr;

		if (end){

			 next_state = sMainMenu; // Cambiamos el estado actual al de menu
			 end = RESET; // Bajamos la bandera

			 //Reseteamos la cola para recibir nuevos comandos
			 xQueueReset(xQueue_InputData);
		}else{

			// El comando recibido solo tener el largo de 1 caracter
			if(cmd->functionType != -1){

				switch (cmd->functionType) {
					case 1:{

						//Envia a imprimir en la consola lo que se debe mostrar en el menu
						xQueueSend(xQueue_Print,&msg_option_1,portMAX_DELAY);
						xQueueReset(xQueue_InputData); // RESETEAMOS LA COLA INPUT PARA ESPERAR NUEVOS COMANDOS

						// Aca se deberia notificar para cambiar la variable next_state y notification
						next_state = sGo;
						xTaskNotify(xHandleTask_Go, 0 ,eNoAction); // NOS VAMOS AL ESTADO sGo


						break;
					}case 2:{

						//Envia a imprimir en la consola lo que se debe mostrar en el menu
						xQueueSend(xQueue_Print,&msg_option_2,portMAX_DELAY);
						xQueueReset(xQueue_InputData); // RESETEAMOS LA COLA INPUT PARA ESPERAR NUEVOS COMANDOS

						// Aca se deberia notificar para cambiar la variable next_state y notification
						next_state = sGoTo;
						xTaskNotify(xHandleTask_GoTo, 0 ,eNoAction); // NOS VAMOS AL ESTADO sGo


						break;
					}case 3:{
						//Envia a imprimir en la consola lo que se debe mostrar en el menu
						xQueueSend(xQueue_Print,&msg_option_2,portMAX_DELAY);

						// Aca se deberia notificar para cambiar la variable next_state y notification
						next_state = sMainMenu;
						xTaskNotify(xHandleTask_Menu,0,eNoAction);



						break;
					}
					default:{
						///////
						continue;

					}
				}

			}else{
				xQueueSend(xQueue_Print, &msg_invalid,portMAX_DELAY);
				//Aca se deberia notificar cambiar la variable next_state y notificar
				wrong_command = RESET; // RESETEAMOS LA BANDERA
				xQueueReset(xQueue_InputData); // Reseteamos la cola que recibe los comandos
				memset(cmd->payload,0,sizeof(cmd->payload)); // Limpiamos el payload

				xTaskNotify(xHandleTask_Menu,0,eNoAction); // Notificamos a la funcion menu para que pueda inmediatamente mandar de nuevo el menu
				xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
			}
		}



	}// Fin del loop de esta tarea

}


void vTask_Commands( void * pvParameters ){

	BaseType_t notify_status = {0};
	command_t cmd = {0};

   while(1){

	   //Esperamos la notificacion desde la interrupcion
	   notify_status = xTaskNotifyWait(0,0,NULL,portMAX_DELAY); // Esoerar hasta que la notificacion salte

	   //Cuando es verdadero significa que se recibio una notificacion
	   if (notify_status == pdPASS){

		   process_command(&cmd);

	   }
   }
}

//////////////////////////////////////////////////////////////////////// MENU STATE/////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//---------------------------------------------------------------------------------------------------------------------------------------------------------------//


////////////////////////////////////////////////////////////////////////STOP STATE/////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void vTask_Stop( void * pvParameters ){

	while(1){
		 //Esperamos la notificacion desde la interrupcion de comandos
		 xTaskNotifyWait(0,0,NULL,portMAX_DELAY); // Esoerar hasta que la notificacion salte

		 // Este comando lo que busca es apagar el robot y detenerlo de su estado de movimiento
		stop(handler_Motor_Array); // Apagamos los motores
		stopTimer(&handlerTIM2_PARAMETROS_MOVIMIENTO); // Detenemos los muestreos

		// Bajamos las banderas de movimiento alguno
		flag_Go_Straigh   = RESET;
		flag_GoTo_Straigh = RESET;
		flag_Roll         = RESET;
		flag_RollTo       = RESET;

		 if (end){
			 xTaskNotify(xHandleTask_Menu,0, eNoAction);
		 }


	}
}

////////////////////////////////////////////////////////////////////////STOP STATE/////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//---------------------------------------------------------------------------------------------------------------------------------------------------------------//


////////////////////////////////////////////////////////////////////////GO STATE/////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void vTask_Go( void * pvParameters ){


	while(1){

		//Esperamos la notificacion desde la interrupcion de comandos
		 xTaskNotifyWait(0,0,NULL,portMAX_DELAY); // Esoerar hasta que la notificacion salte

		// Si estamos aqui se quiere solo que el robot vaya hacia adelante y el linea recta
		Mode_dir.Mode = Line;
		Mode_dir.direction_s_r = fparam;

		resetParameters();

		On_motor_Straigh_Roll(handler_Motor_Array,  Mode_dir); // Encendemos los motores para irnos hacia adelante y con una velocidad fija
		startTimer(&handlerTIM2_PARAMETROS_MOVIMIENTO); // Comenzamos el muestreo de datos con los que aplicaremos un control adecuado

	}


}


void vTask_Control( void * pvParameters ){

	while(1){

		//Esperamos la notificacion desde la interrupcion de comandos
		 xTaskNotifyWait(0,0,NULL,portMAX_DELAY); // Esoerar hasta que la notificacion salte

		 switch (next_state) {
			case sGo:{

				go(handler_Motor_Array,
				  &handler_MPUAccel_6050,
				  &parameters_Pos_Robot,
				  &parameters_Path_Robot,
				  &parameter_PID_distace,
				  cal_Gyro,
				  &flag_angulo,
				  &flag_measurements,
				  &flag_control,
				  userMsg,
				  Mode_dir); // Esta funcion se ejecutara cada 16ms, tiempo entre interrupciones del Timer 2

				break;
			}case sGoTo:{


				distance_to_go = distance_traveled(&parameters_Path_Robot, parameters_Pos_Robot.xg_position, parameters_Pos_Robot.yg_position);

				// Función de control del robot
				go(handler_Motor_Array,
				  &handler_MPUAccel_6050,
				  &parameters_Pos_Robot,
				  &parameters_Path_Robot,
				  &parameter_PID_distace,
				  cal_Gyro,
				  &flag_angulo,
				  &flag_measurements,
				  &flag_control,
				  userMsg,
				  Mode_dir); // Esta funcion se ejecutara cada 16ms, tiempo entre interrupciones del Timer 2

				if (!(abs(distance_to_go) < parameters_Path_Robot.line_Distance)){
					// Paramos el proceso
					end = SET;
					xTaskNotify(xHandleTask_Stop,0, eNoAction); // Levantamos la tarea de stop para parar la ejecución
				}


				break;
			}
			default:{break;}
		}
	}

}

////////////////////////////////////////////////////////////////////////GO STATE/////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//---------------------------------------------------------------------------------------------------------------------------------------------------------------//


////////////////////////////////////////////////////////////////////////GOTO STATE/////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void vTask_GoTo( void * pvParameters ){

	// En esta tarea querremos ir solo hacia un punto deseado por el usuario

	while(1){

		//Esperamos la notificacion desde la interrupcion de comandos
		 xTaskNotifyWait(0,0,NULL,portMAX_DELAY); // Esoerar hasta que la notificacion salte

		// Si estamos aqui se quiere solo que el robot vaya hacia adelante y el linea recta
		Mode_dir.Mode = Line;
		Mode_dir.direction_s_r = fparam;

		// Almacenamos la distancia en milimetros a recorrer
		parameters_Path_Robot.line_Distance = sparam;

		resetParameters();

		On_motor_Straigh_Roll(handler_Motor_Array,  Mode_dir); // Encendemos los motores para irnos hacia adelante y con una velocidad fija

		// seteamos la posicion inicial como la posicion actual global del robot
		parameters_Path_Robot.start_position_x = parameters_Pos_Robot.xg_position;
		parameters_Path_Robot.start_position_y = parameters_Pos_Robot.yg_position;

		// seteamos la posicion final usando parametros polares

		//Usando el angulo actual global con respecto al eje x se tiene que
		parameters_Path_Robot.goal_Position_x = parameters_Path_Robot.line_Distance * cos(parameters_Pos_Robot.rad_global) + parameters_Path_Robot.start_position_x ; // usando la funcion coseno para hallar la coordenada x de llegada
		parameters_Path_Robot.goal_Position_y = parameters_Path_Robot.line_Distance * sin(parameters_Pos_Robot.rad_global) + parameters_Path_Robot.start_position_y ; //usando la funcion coseno para hallar la coordenada y de llegada

		// definimos los parametros del camino en funcion de la situacion actual
		calculation_parameter_distance(&parameters_Path_Robot);

		startTimer(&handlerTIM2_PARAMETROS_MOVIMIENTO); // Comenzamos el muestreo de datos con los que aplicaremos un control adecuado

	}

}



////////////////////////////////////////////////////////////////////////GOTO STATE/////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





void vTask_Print( void * pvParameters ){


	uint32_t *msg;

   while(1){

	   xQueueReceive(xQueue_Print, &msg, portMAX_DELAY);
	   //usart write command
	   writeMsg(&handlerUSART, (char*) msg);
   }
}

void process_command (command_t *cmd){

	extract_command(cmd);


	if (next_state == sMainMenu){
		//Notificamos a la tarea respectiva
		xTaskNotify(xHandleTask_Menu,(uint32_t)cmd, eSetValueWithOverwrite);
	}else if (!wrong_command){
		end = SET;
		//Notificamos a la tarea en el estado de parada.
		xTaskNotify(xHandleTask_Stop,0, eNoAction);
	}else{
		//Notificamos a la tarea respectiva
		xTaskNotify(xHandleTask_Menu,(uint32_t)cmd, eSetValueWithOverwrite);
	}


}


int extract_command (command_t *cmd){

	uint8_t item;
	uint8_t counter_j = 0;
	BaseType_t status;

	status = uxQueueMessagesWaiting(xQueue_InputData);
	if (status == 0){
		return -1;
	}

	if (wrong_command){
				// si se llego aqui es porque se mando un comando erronio, por lo que hay resetear y mandar de nuevo
				cmd->functionType = -1;
				memset(cmd->payload,0,sizeof(cmd->payload)); // Limpiamos el payload
				xQueueReset(xQueue_InputData);
	}else{

		do{
			// Recibimos un elemento y lo montamos en el item ademas no deseamos bloquarlo
			status = xQueueReceive(xQueue_InputData, &item,0);
			if(status == pdTRUE){

				//vamos llenando el arreglo del comando
				cmd->payload[counter_j++] = item;

			}
		}while(item != '#');

		cmd->payload[counter_j] = '\0'; // Agregamos la terminacion nula para que tengamos un string

		// Del comando entregado extraemos toda la informacion necesaria para poder usarla luego en los estados necesarios
	//	sscanf((char *) cmd->payload, "%s %u %u %u %s", data ,&firstParameter, &secondParameter, &thirdParameter, userMsg);

		extract_info(cmd, data, firstParameter, secondParameter, thirdParameter, &fparam, &sparam, &tparam);

		memset(cmd->payload,0,sizeof(cmd->payload)); // Limpiamos el payload
		xQueueReset(xQueue_InputData);

		if (strcmp(data, "sGo") == 0){
			cmd->functionType = 1;
		}
		else if (strcmp(data, "sGoTo") == 0){
			cmd->functionType = 2;
		}

	}



	return 0;

}

// INTERRUPCIONES DE EXTI
void callback_extInt1(void){
	// Aumentamos las cuentas
	handler_Motor_Array[0]->parametersMotor.counts++;
}

void callback_extInt3(void){
	// Aumentamos las cuentas
	handler_Motor_Array[1]->parametersMotor.counts++;
}




//Interripcion USART2
void usart1Rx_Callback(void){

	rxData = getRxData();
	writeChar(&handlerUSART, rxData);

	if (rxData == '\r'){
		wrong_command = SET;
	}
	BaseType_t xHigerPriorituTaskWoken;
	(void) xHigerPriorituTaskWoken;
	xHigerPriorituTaskWoken = pdFALSE;

	//Verificamos que la cola aun no se encuentra llena
	xReturned = xQueueIsQueueFullFromISR(xQueue_InputData);
	// Si retorna que aun tiene espacio entoncesretorna falso

	if (xReturned != pdTRUE ){

		xQueueSendToBackFromISR(xQueue_InputData,
								(void*) &rxData,
								NULL);

	}else{


		if (rxData == '#'){

			xQueueReceiveFromISR(xQueue_InputData,
								 (void *) &rxData,
								 NULL);
			xQueueSendToBackFromISR(xQueue_InputData,
									(void *) &rxData
									,NULL);
		}

	}

	if (rxData == '#' || rxData == '\r'){
		// Se manda la notificacion de la tarea que se quiere mover al estado de RUN
		xTaskNotifyFromISR(xHandleTask_Commands,
						   0,
						   eNoAction,
						   NULL);
//		xSemaphoreGiveFromISR(xSemaphore_Handle, &xHigerPriorituTaskWoken);

	}
}

//Definimos la funcion que se desea ejecutar cuando se genera la interrupcion por el TIM2

void BasicTimer2_Callback(void){

	// Levantamos bandera que calcula el angulo actual
	flag_angulo = SET;


	// Levantamos la bandera que corresponde con los calculos  odometricos del robot, como la distancia
	// Recorrida, la posicion actual y la velocidad

	//----------------Accion a realizar con un tiempo especifico--------------------
	if(counting_action >= timeAction_TIMER_Sampling){
			flag_measurements = SET;
	}else{ counting_action++; }

	xTaskNotifyFromISR(xHandleTask_Control,
					   0,
					   eNoAction,
					   NULL);

	// EN LA FUNCION 'GO' ESTAN LAS DOS BANDERAS SE ANALIZARAN Y SE EJECUTARAN
}


//Definicion de funciones varias


// Calibracion Gyros:

float calibracionGyros (MPUAccel_Config *ptrMPUAccel, uint8_t axis){

	uint16_t  numMedidas = 200;
	float     medidas    = 0;
	float     suma       = 0;
	uint8_t   contador   = 0;
	float     promedio   = 0;

	switch (axis) {
		case 0:{
			while (contador < numMedidas){
				medidas = readGyro_X(ptrMPUAccel);
				suma += medidas;
				contador++;
				delay_ms(1); // esperamos 1 milisegundo
			}
			promedio = suma / numMedidas;
			break;
		}case 1:{
			while (contador < numMedidas){
				medidas = readGyro_Y(ptrMPUAccel);
				suma += medidas;
				contador++;
				delay_ms(1); // esperamos 1 milisegundo
			}
			promedio = suma / numMedidas;
			break;
		}case 2:{
			while (contador < numMedidas){
				medidas = readGyro_Z(ptrMPUAccel);
				suma += medidas;
				contador++;
				delay_ms(1); // esperamos 1 milisegundo
			}
			promedio = suma / numMedidas;
			break;
		}default:{
			break;
		}
	}


	return promedio;
}


void getAngle(MPUAccel_Config *ptrMPUAccel,float angle_init, double calibr, Parameters_Position_t *ptrParameter_position){

	///////////////////////////MEDIDA DEL ANGULO ACUMULADO////////////////////////////////////

	//----------------Accion a Realiza cada interrupción------------------
		//Leemos el ángulo
		//Lectura velocidad angular
		float w = readGyro_Z(ptrMPUAccel) - calibr;
		//Calculo angulo
		float ang_d = angle_init + (w * 16)/1000; // conversion de velocidad angular a grados absolutos con respecto al inicio del programa

		ptrParameter_position->grad_relativo = ang_d;

		//Acumulamos los angulos
		sum_ang += ptrParameter_position->grad_relativo;
		//Se acumula el tiempo
		time_accumulated += handlerTIM2_PARAMETROS_MOVIMIENTO.TIMx_Config.TIMx_period;
}

void get_measuremets_parameters(Motor_Handler_t *ptrMotorHandler[2], Parameters_Position_t *ptrParameter_position, state_dir_t operation_mode_dir){

	//Verificamos el modo
	if(operation_mode_dir.Mode == Line){ // Levantamos la vandera que calcula todos los parametros necesarios para el control

			//Guardamos el tiempo entre acciones especificas
			time_accion = time_accumulated;
			//Calculamos el angulo promedio y la establecemos como el angulo relativo
			promAng = sum_ang / counting_action;
			ptrParameter_position->rad_relativo = (promAng * M_PI) / 180; //[rad]

			ptrParameter_position->grad_global += atan2(sin((sum_ang * M_PI) / 180),
					  	  	  	  	  	  	  	  	    cos((sum_ang * M_PI) / 180)) * (180 / M_PI); //[°] angulo acumulado global en grados

			ptrParameter_position->rad_global = atan2(sin((ptrParameter_position->grad_global * M_PI) / 180),
													  cos((ptrParameter_position->grad_global * M_PI) / 180)); //[rad]

			// Con la siguiente accion conseguimos que el angulo que deseamos solo este dentro del rango [-pi,pi]
			ptrParameter_position->rad_relativo = atan2(sin(ptrParameter_position->rad_relativo),
														cos(ptrParameter_position->rad_relativo));

			//Calculamos la velocidad
			if (operation_mode_dir.direction_s_r == SET){

				// SI estamos aqui se tiene una direccion deseada hacia adelante
				// almacenamos las variables de velocidad y dirección en función de hacia donde se este llendo

				ptrMotorHandler[0]->parametersMotor.dis = (paso_mm_1 * ptrMotorHandler[0]->parametersMotor.counts);// Calculamos la distancia recorrida contando cuantos pasos a dado la rueda izquierda //[mm]
				ptrMotorHandler[1]->parametersMotor.dis = (paso_mm_2 * ptrMotorHandler[1]->parametersMotor.counts);// Calculamos la distancia recorrida contando cuantos pasos a dado la rueda derecha   //[mm]
				ptrMotorHandler[0]->parametersMotor.vel = ptrMotorHandler[0]->parametersMotor.dis / time_accion; // Calculamos la velocidad de la rueda izquierda     //[m/s]
				ptrMotorHandler[1]->parametersMotor.vel = ptrMotorHandler[1]->parametersMotor.dis / time_accion; // Calculamos la velocidad de la rueda derecha    //[m/s]

			}else{
				// SI estamos aqui se tiene una direccion deseada hacia atras
				// almacenamos las variables de velocidad y dirección en función de hacia donde se este llendo

				ptrMotorHandler[0]->parametersMotor.dis = -(paso_mm_1 * ptrMotorHandler[0]->parametersMotor.counts);// Calculamos la distancia recorrida contando cuantos pasos a dado la rueda izquierda //[mm]
				ptrMotorHandler[1]->parametersMotor.dis = -(paso_mm_2 * ptrMotorHandler[1]->parametersMotor.counts);// Calculamos la distancia recorrida contando cuantos pasos a dado la rueda derecha   //[mm]
				ptrMotorHandler[0]->parametersMotor.vel = -ptrMotorHandler[0]->parametersMotor.dis / time_accion; // Calculamos la velocidad de la rueda izquierda     //[m/s]
				ptrMotorHandler[1]->parametersMotor.vel = -ptrMotorHandler[1]->parametersMotor.dis / time_accion; // Calculamos la velocidad de la rueda derecha    //[m/s]

			}

			//Reiniciamos el numero de conteos
			ptrMotorHandler[0]->parametersMotor.counts = 0;
			ptrMotorHandler[1]->parametersMotor.counts = 0;

			//Reiniciamos variable
			sum_ang = 0;

			//Reiniciamos tiempo
			time_accumulated = 0;

			//Reiniciamos el contador de accion
			counting_action = 0;

	}
	else if(operation_mode_dir.Mode == Roll)
	{
			//Guardamos el tiempo entre acciones especificas
			time_accion = time_accumulated;
			//Calculamos el angulo promedio y la establecemos como el angulo relativo
			promAng = sum_ang / counting_action;
			ptrParameter_position->rad_relativo = (promAng * M_PI) / 180; //[rad]

			ptrParameter_position->grad_global += atan2(sin((sum_ang * M_PI) / 180),
					  	  	  	  	  	  	  	  	    cos((sum_ang * M_PI) / 180)) * (180 / M_PI); //[°] angulo acumulado global en grados

			ptrParameter_position->rad_global = atan2(sin((ptrParameter_position->grad_global * M_PI) / 180),
													  cos((ptrParameter_position->grad_global * M_PI) / 180)); //[rad]

			// Con la siguiente accion conseguimos que el angulo que deseamos solo este dentro del rango [-pi,pi]
			ptrParameter_position->rad_relativo = atan2(sin(ptrParameter_position->rad_relativo),
														cos(ptrParameter_position->rad_relativo));

			//Calculo de la distancia recorrida por cada rueda

			if (operation_mode_dir.direction_s_r == SET){

				// SI estamos aqui se tiene un una direccion de giro CW
				// almacenamos las variables de velocidad y dirección en función de hacia donde se este girando

				ptrMotorHandler[0]->parametersMotor.dis = (paso_mm_1 * ptrMotorHandler[0]->parametersMotor.counts);// Calculamos la distancia recorrida contando cuantos pasos a dado la rueda izquierda //[mm]
				ptrMotorHandler[1]->parametersMotor.dis = -(paso_mm_2 * ptrMotorHandler[1]->parametersMotor.counts);// Calculamos la distancia recorrida contando cuantos pasos a dado la rueda derecha   //[mm]
				ptrMotorHandler[0]->parametersMotor.vel = ptrMotorHandler[0]->parametersMotor.dis / time_accion; // Calculamos la velocidad de la rueda izquierda     //[m/s]
				ptrMotorHandler[1]->parametersMotor.vel = -ptrMotorHandler[1]->parametersMotor.dis / time_accion; // Calculamos la velocidad de la rueda derecha    //[m/s]

			}else{
				// SI estamos aqui se tiene un una direccion de giro CCW
				// almacenamos las variables de velocidad y dirección en función de hacia donde se este girando

				ptrMotorHandler[0]->parametersMotor.dis = -(paso_mm_1 * ptrMotorHandler[0]->parametersMotor.counts);// Calculamos la distancia recorrida contando cuantos pasos a dado la rueda izquierda //[mm]
				ptrMotorHandler[1]->parametersMotor.dis = (paso_mm_2 * ptrMotorHandler[1]->parametersMotor.counts);// Calculamos la distancia recorrida contando cuantos pasos a dado la rueda derecha   //[mm]
				ptrMotorHandler[0]->parametersMotor.vel = -ptrMotorHandler[0]->parametersMotor.dis / time_accion; // Calculamos la velocidad de la rueda izquierda     //[m/s]
				ptrMotorHandler[1]->parametersMotor.vel = ptrMotorHandler[1]->parametersMotor.dis / time_accion; // Calculamos la velocidad de la rueda derecha    //[m/s]

			}

			//Reiniciamos el numero de conteos
			ptrMotorHandler[0]->parametersMotor.counts = 0;
			ptrMotorHandler[1]->parametersMotor.counts = 0; // RESETEAMOS LAS CUENTAS

//			//Cálculo ángulo debido al desplazamiento del ICR
//			ang_for_Displament += (((ptrMotorHandler[1]->parametersMotor.dis - ptrMotorHandler[0]->parametersMotor.dis) * 100)
//					               / distanceBetweenWheels)*(180/M_PI); //[°]
//
			//Reiniciamos variable
			sum_ang = 0;
			//Reiniciamos tiempo
			time_accumulated = 0;

			//Reiniciamos el contador de acción
			counting_action  = 0;

		//Combinar ambos ángulos
//		ang_complementary = ptrParameter_position->grad_relativo + ang_for_Displament;
	}
	else{  __NOP(); }

}


void On_motor_Straigh_Roll(Motor_Handler_t *ptrMotorhandler[2], state_dir_t operation_mode_dir){


	if (operation_mode_dir.Mode == Line){
				//Activamos el motor
				// ENCENCEMOS EL MOTOR 1 (LEFT)
					// Seteamos correctamente la direccion de cada motor
					set_direction_straigh_roll(ptrMotorhandler, operation_mode_dir);

					enableOutput(ptrMotorhandler[0]->phandlerPWM);
					GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOEN,SET); // Encendemos el motor 1

					// ENCENCEMOS EL MOTOR 2 (Right)
					//Se enciende el motor 2
					enableOutput(ptrMotorhandler[1]->phandlerPWM);
					GPIO_WritePin_Afopt (ptrMotorhandler[1]->phandlerGPIOEN,SET);


	}else if (operation_mode_dir.Mode == Roll){
				//Activamos el motor
				// ENCENCEMOS EL MOTOR 1 (LEFT)
					set_direction_straigh_roll(ptrMotorhandler, operation_mode_dir);

					//Se enciende el motor 1
					enableOutput(ptrMotorhandler[0]->phandlerPWM);
					GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOEN,SET); // Encendemos el motor 1

					// ENCENCEMOS EL MOTOR 2 (Right)
					//Se enciende el motor 2
					enableOutput(ptrMotorhandler[1]->phandlerPWM);
					GPIO_WritePin_Afopt (ptrMotorhandler[1]->phandlerGPIOEN,SET);


	}

}

void set_direction_straigh_roll (Motor_Handler_t *ptrMotorhandler[2], state_dir_t operation_mode_dir){

	// Esta funcion setea correctamente la direccion de los motores dependiendo de lo que se quiera.
	if (operation_mode_dir.Mode == Line){
		// Si estamos aqui es porque queremos Setear la direccion hacia adelante o hacia atras

		// Si queremos ir hacia adelante

		// Primero revisamos en que direccion se encuentra el robot para ver si si se aplica
		// el cambio o no
		if ((ptrMotorhandler[0]->configMotor.dir != operation_mode_dir.direction_s_r)){
			// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
			ptrMotorhandler[0]->configMotor.dir = operation_mode_dir.direction_s_r;
			GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOIN, !ptrMotorhandler[0]->configMotor.dir); // La direccion estaba en RESET, la cambiamos a SET
			PWMx_Toggle(ptrMotorhandler[0]->phandlerPWM);

		}

		if ((ptrMotorhandler[1]->configMotor.dir != operation_mode_dir.direction_s_r)){
			// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
			ptrMotorhandler[1]->configMotor.dir = operation_mode_dir.direction_s_r;
			GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOIN,!ptrMotorhandler[1]->configMotor.dir); // La direccion estaba en RESET, la cambiamos a SET
			PWMx_Toggle(ptrMotorhandler[1]->phandlerPWM);
		}
		// Puede que no analice ningun if y simplemente no haga nada

	}else if (operation_mode_dir.Mode == Roll){
		// si estamos aca es porque queremos cambiar la direccion de rotacion, Solo en el caso requerido

			// Si queremos ir en centido CW o horario

			// Primero revisamos en que direccion se encuentra el robot para ver si si se aplica
			// el cambio o no
			if ((ptrMotorhandler[0]->configMotor.dir == operation_mode_dir.direction_s_r)){
				// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
				ptrMotorhandler[0]->configMotor.dir  =  !operation_mode_dir.direction_s_r;
				GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOIN, !ptrMotorhandler[0]->configMotor.dir); // La direccion estaba en RESET, la cambiamos a SET
				PWMx_Toggle(ptrMotorhandler[0]->phandlerPWM);

			}

			if ((ptrMotorhandler[1]->configMotor.dir != operation_mode_dir.direction_s_r)){
				// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
				ptrMotorhandler[1]->configMotor.dir = operation_mode_dir.direction_s_r;
				GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOIN, !ptrMotorhandler[1]->configMotor.dir); // La direccion estaba en SET, la cambiamos a RESET
				PWMx_Toggle(ptrMotorhandler[1]->phandlerPWM);
			}
			// Puede que no analice ningun if y simplemente no haga nada


	}
}

void change_dir_straigh_Roll(Motor_Handler_t *ptrMotorhandler[2], state_dir_t operation_mode_dir){

	if (operation_mode_dir.Mode == Line){
		// Si estamos aqui es porque queremos cambiar la direccion en linea recta correctamente

		// antes de cambiar la direccion apagamos los motores
		GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOEN,RESET);
		GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN,RESET);

		// Primero revisamos en que direccion se encuentra el robot para ver si si se aplica
		// el cambio o no
		if ((ptrMotorhandler[0]->configMotor.dir != operation_mode_dir.direction_s_r)){
			// si estamos aqui es porque se quiere cambiar la direccion del robot

			// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
			ptrMotorhandler[0]->configMotor.dir = operation_mode_dir.direction_s_r;
			GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOIN, !ptrMotorhandler[0]->configMotor.dir); // La direccion estaba en RESET, la cambiamos a SET
			PWMx_Toggle(ptrMotorhandler[0]->phandlerPWM);

		}

		if ((ptrMotorhandler[1]->configMotor.dir != operation_mode_dir.direction_s_r)){
			// si estamos aqui es porque se quiere cambiar la direccion del robot
			ptrMotorhandler[1]->configMotor.dir = operation_mode_dir.direction_s_r;
			// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
			GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOIN, !ptrMotorhandler[1]->configMotor.dir); // La direccion estaba en RESET, la cambiamos a SET
			PWMx_Toggle(ptrMotorhandler[1]->phandlerPWM);
		}
		// Puede que no analice ningún if y simplemente no haga nada


		// volvemos a encender los motores
		GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOEN,SET);
		GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN,SET);





	}else if (operation_mode_dir.Mode == Roll){
		// si estamos aca es porque queremos cambiar la direccion de rotacion

			// Si queremos ir en centido CW o horario

			// Primero revisamos en que direccion se encuentra el robot para ver si si se aplica
			// el cambio o no
			// si estamos aqui es porque se quiere cambiar la direccion DEL ROBOT A CW

			// antes de cambiar la direccion apagamos los motores
			GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOEN,RESET);
			GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN,RESET);

			if ((ptrMotorhandler[0]->configMotor.dir == operation_mode_dir.direction_s_r)){
				// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
				ptrMotorhandler[0]->configMotor.dir = !operation_mode_dir.direction_s_r;
				GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOIN, !ptrMotorhandler[0]->configMotor.dir ); // La direccion estaba en RESET, la cambiamos a SET
				PWMx_Toggle(ptrMotorhandler[0]->phandlerPWM);

			}

			if ((ptrMotorhandler[1]->configMotor.dir != operation_mode_dir.direction_s_r)){
				// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
				ptrMotorhandler[1]->configMotor.dir = operation_mode_dir.direction_s_r;
				GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOIN, !ptrMotorhandler[1]->configMotor.dir); // La direccion estaba en SET, la cambiamos a RESET
				PWMx_Toggle(ptrMotorhandler[1]->phandlerPWM);
			}
			// Puede que no analice ningun if y simplemente no haga nada


			// volvemos a encender los motores
			GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOEN,SET);
			GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN,SET);

	}
}


void stop (Motor_Handler_t *ptrMotorhandler[2]){

	//DESACTIVAMOS EL MOTOR
	// APAGAMOS EL MOTOR 1 (LEFT)
		//Se enciende el motor 1
		disableOutput(ptrMotorhandler[0]->phandlerPWM);
		GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOEN, RESET); // Apagamos el motor 1
		// APAGAMOS EL MOTOR 2 (Right)
		//Se enciende el motor 2
		disableOutput(ptrMotorhandler[1]->phandlerPWM);
		GPIO_WritePin_Afopt (ptrMotorhandler[1]->phandlerGPIOEN,RESET);


}


void int_Config_Motor(Motor_Handler_t *ptrMotorhandler[2],
		              Parameters_Position_t *ptrPosHandler,
					  Parameters_Path_t *ptrPathHandler ,
					  PID_Parameters_t *ptrPIDHandler){

	//---------------Motor Izquierdo----------------
	ptrMotorhandler[0] = &handlerMotor1_t;

	//Parametro de la señal del dutty
	ptrMotorhandler[0]->configMotor.dutty =  fixed_dutty;
	//handler de los perifericos
	ptrMotorhandler[0]->phandlerGPIOEN = &handlerEn1PinC10;
	ptrMotorhandler[0]->phandlerGPIOIN = &handlerIn1PinC12;
	ptrMotorhandler[0]->phandlerPWM = &handlerPWM_1;
	//definicion de parametros
	ptrMotorhandler[0]->parametersMotor.pid->e0 = 0;
	ptrMotorhandler[0]->parametersMotor.pid->e_prev = 0;
	ptrMotorhandler[0]->parametersMotor.pid->u = 0;
	ptrMotorhandler[0]->parametersMotor.pid->e_int = 0;
	//Calculo de Constantes PID
	ptrMotorhandler[0]->parametersMotor.pid->kp = 250;
	ptrMotorhandler[0]->parametersMotor.pid->ki = 0;
	ptrMotorhandler[0]->parametersMotor.pid->kd = 100;

	//---------------Motor Derecho----------------
	//Parametro de la señal del dutty
	ptrMotorhandler[1] = &handlerMotor2_t;

	ptrMotorhandler[1]->configMotor.dutty =  fixed_dutty;
	//handler de los perifericos
	ptrMotorhandler[1]->phandlerGPIOEN = &handlerEn2PinC11;
	ptrMotorhandler[1]->phandlerGPIOIN = &handlerIn2PinD2;
	ptrMotorhandler[1]->phandlerPWM = &handlerPWM_2;
	//definicion de parametros
	ptrMotorhandler[1]->parametersMotor.pid->e0 =  0;
	ptrMotorhandler[1]->parametersMotor.pid->e_prev = 0;
	ptrMotorhandler[1]->parametersMotor.pid->u =  0;
	ptrMotorhandler[1]->parametersMotor.pid->e_int = 0;
	//Calculo de Constantes PID
	ptrMotorhandler[1]->parametersMotor.pid->kp = 250;
	ptrMotorhandler[1]->parametersMotor.pid->ki = 0;
	ptrMotorhandler[1]->parametersMotor.pid->kd = 100;

	//---------------PID del la distancia-----------------
	//definicion de parametros
	ptrPIDHandler->e0 = ptrPIDHandler->e_prev = 0;
	ptrPIDHandler->u =  ptrPIDHandler->e_int = 0;
	//Calculo de Constantes PID
	ptrPIDHandler->kp = 1.0;
	ptrPIDHandler->ki = 0.1;
	ptrPIDHandler->kd = 0.8;

	//-------------- Parametros de posicion---------------
	ptrPosHandler->grad_global   = 0;
	ptrPosHandler->rad_global    = 0;
	ptrPosHandler->grad_relativo = 0;
	ptrPosHandler->rad_relativo  = 0;
	ptrPosHandler->xr_position   = 0;
	ptrPosHandler->yr_position   = 0;
	ptrPosHandler->xg_position   = ptrPosHandler->xg_position_inicial = 0;
	ptrPosHandler->yg_position   = ptrPosHandler->yg_position_inicial = 0;


	//--------------Parametros de Path-----------------
	ptrPathHandler->angle = 0;
	ptrPathHandler->goal_Position_x = ptrPathHandler->goal_Position_y = 0;
	ptrPathHandler->line_Distance = 0;
	ptrPathHandler->start_position_x = ptrPathHandler->start_position_y = 0;

}


int goTo(Motor_Handler_t *ptrMotorhandler[2],
		 MPUAccel_Config *ptrMPUhandler,
		 Parameters_Position_t *ptrPosHandler ,
		 Parameters_Path_t *ptrPathHandler,
		 PID_Parameters_t *ptrPIDHandler,
		 double calib ,
		 uint8_t *fAnglulo,
		 uint8_t *fMeasurements,
		 uint8_t *fcontrol,
		 char buff[64],
		 uint32_t distance_mm ,
		 state_dir_t operation_mode_dir){

	// esta funcion se encarga de enviar al robot en una linea recta hacia una distancia especifica
	// Para ello lo que se hara es simplemente encender el robot y al mismo tiempo calcular su distancia recorrida
	double distance_to_go = 0;
	uint8_t done = RESET;

	// seteamos la posicion inicial como la posicion actual global del robot
	ptrPathHandler->start_position_x = ptrPosHandler->xg_position;
	ptrPathHandler->start_position_y = ptrPosHandler->yg_position;

	// seteamos la posicion final usando parametros polares

	//Usando el angulo actual global con respecto al eje x se tiene que
	ptrPathHandler->goal_Position_x = distance_mm * cos(ptrPosHandler->rad_global) + ptrPathHandler->start_position_x ; // usando la funcion coseno para hallar la coordenada x de llegada
	ptrPathHandler->goal_Position_y = distance_mm * sin(ptrPosHandler->rad_global) + ptrPathHandler->start_position_y ; //usando la funcion coseno para hallar la coordenada y de llegada

	// definimos los parametros del camino en funcion de la situacion actual
	calculation_parameter_distance(ptrPathHandler);

	On_motor_Straigh_Roll(ptrMotorhandler, operation_mode_dir); // Encendemos el robot en la direccion deseada

	while(!done){
		// calculamos la distancia con la libreria PosRobt.h

		distance_to_go = distance_traveled( ptrPathHandler, ptrPosHandler->xg_position, ptrPosHandler->yg_position);

		// Función de control del robot
		go(ptrMotorhandler,
		   ptrMPUhandler,
		   ptrPosHandler,
		   ptrPathHandler,
		   ptrPIDHandler,
		   calib,
		   fAnglulo,
		   fMeasurements,
		   fcontrol,
		   buff,
		   operation_mode_dir); // Con esta funcion hacemos que el robot simplemente se mueva

		if (!(distance_to_go < distance_mm)){
			// Paramos el proceso
			done = !done;
		}

		// Observamos si hay algun comando en espera
		fillComand();

	}


	return done;
}


void PID_control(Motor_Handler_t *ptrMotorhandler[2] ,
		        Parameters_Path_t *ptrPathHandler,
				Parameters_Position_t *ptrPosHandler,
				PID_Parameters_t *ptrPIDHandler){

	//Conversion de tiempo
	float sampling_time = ((float) (handlerTIM2_PARAMETROS_MOVIMIENTO.TIMx_Config.TIMx_period * timeAction_TIMER_Sampling) / 1000); //[s]

	//Control PID para la distancia
	float distance_recta = (distance_to_straight_line(ptrPathHandler, ptrPosHandler->xg_position, ptrPosHandler->yg_position)) / 1000; //[m]

	//Aplicacion del PID par el control de la distancia del robot al centro
	PID_calc(ptrPIDHandler, sampling_time, 0,  distance_recta);

	//Aplicacndo correcion
	vel_Setpoint_1 = velSetPoint - ptrPIDHandler->u; // Cambio en la velocidad de la rueda izquierda
	vel_Setpoint_2 = velSetPoint + ptrPIDHandler->u; // cambio en la velocidad de la rueda derecha

	//Aplicacion del PID par el control de las velocidades
	PID_calc(ptrMotorhandler[0]->parametersMotor.pid, sampling_time, vel_Setpoint_1,  ptrMotorhandler[0]->parametersMotor.vel); // Accion de control 1
	PID_calc(ptrMotorhandler[1]->parametersMotor.pid, sampling_time, vel_Setpoint_2,  ptrMotorhandler[1]->parametersMotor.vel); // Accion de control 2

	//Cambiamos valores
	ptrMotorhandler[0]->configMotor.new_dutty += ptrMotorhandler[0]->parametersMotor.pid->u;
	ptrMotorhandler[1]->configMotor.new_dutty += ptrMotorhandler[1]->parametersMotor.pid->u;

	//Correccion del dutty
	// Primero nos aseguramos de la no saturacion de los motores
	if (ptrMotorhandler[0]->configMotor.new_dutty >= 60){

		ptrMotorhandler[0]->configMotor.new_dutty = 60;// El limite superior sera 60 de dutty

	}else if (ptrMotorhandler[0]->configMotor.new_dutty <= fixed_dutty -5){

		ptrMotorhandler[0]->configMotor.new_dutty = fixed_dutty -5; // el limite inferior seria 5 puntos menos al dutty fijo
	}

	if (ptrMotorhandler[1]->configMotor.new_dutty >= 60){

		ptrMotorhandler[1]->configMotor.new_dutty = 60;// El limite superior sera 60 de dutty

	}else if (ptrMotorhandler[1]->configMotor.new_dutty <= fixed_dutty -5){

		ptrMotorhandler[1]->configMotor.new_dutty = fixed_dutty -5; // el limite inferior seria 5 puntos menos al dutty fijo
	}

}

void PID_calc(PID_Parameters_t *ptrPIDHandler,
		      float time_of_sampling,
			  float setpoint,
			  float current_measure){

	//Calculo del error
	ptrPIDHandler->e0 = setpoint-current_measure;
    // Controle PID
	float P =  ptrPIDHandler->kp*ptrPIDHandler->e0; // control proporcional
	ptrPIDHandler->e_int +=  ptrPIDHandler->e0 * time_of_sampling;
	float I = ptrPIDHandler->ki * ptrPIDHandler->e_int; // Control integral
	float D =  ptrPIDHandler->kd*(ptrPIDHandler->e0 - ptrPIDHandler->e_prev) / time_of_sampling; // control derivativo
	ptrPIDHandler->u = P + I + D;
     //Actualizamos el error
	ptrPIDHandler->e_prev = ptrPIDHandler->e0;
}


void go(Motor_Handler_t *ptrMotorhandler[2],
		MPUAccel_Config *ptrMPUhandler,
		Parameters_Position_t *ptrPosHandler ,
		Parameters_Path_t *ptrPathHandler,
		PID_Parameters_t *ptrPIDHandler,
		double calib ,
		uint8_t *fAnglulo,
		uint8_t *fMeasurements,
		uint8_t *fcontrol,
		char buff[64],
		state_dir_t operation_mode_dir){

	////////////////////////////////////////BLOQUE DE MEDICION Y CONTROL//////////////////////////////////////////////////////


	//En este primera medicion se mide el el angulo actual del robot con respecto a una referencia.
	if (*fAnglulo){ // este se ejecutara cada periodo

		// Medimos el angulo actual
		getAngle(ptrMPUhandler, 0, calib, ptrPosHandler);
		// bajamos la bandera
		*fAnglulo = RESET;
	}
	// En la siguiente medicion medimos todos los parametros necesarios para el control posterior
	if (*fMeasurements){ // Este se ejecutara cada (periodo * 13 cuentas)

		// Medimos el angulo actual
		get_measuremets_parameters(ptrMotorhandler, ptrPosHandler, operation_mode_dir);
		// bajamos la bandera
		*fMeasurements = RESET;
		//Levandamos la bandera de control
		*fcontrol = SET;
	}

	// Control
	if (*fcontrol){

		//Calculo odometria
		double distance_prom = (ptrMotorhandler[1]->parametersMotor.dis + ptrMotorhandler[0]->parametersMotor.dis)/2;//[mm]

		ptrPosHandler->xr_position = distance_prom * (cos(ptrPosHandler->rad_global));        //[mm]
		ptrPosHandler->yr_position = distance_prom * (sin(ptrPosHandler->rad_global));       //[mm]

		//Paso de c.relativa a c.globales
		ptrPosHandler->xg_position +=  ptrPosHandler->xr_position;
		ptrPosHandler->yg_position +=  ptrPosHandler->yr_position;

		//Convertimos el valor y imprimimos en la terminal
//		sprintf(buff,"&%#.4f\t%#.4f\n", ptrPosHandler->xg_position , ptrPosHandler->yg_position);
//
//		writeMsg(&handlerUSART, buff);

		PID_control(ptrMotorhandler, ptrPathHandler, ptrPosHandler, ptrPIDHandler);

		*fcontrol = RESET;
	}
}


void roll(Motor_Handler_t *ptrMotorhandler[2],
		MPUAccel_Config *ptrMPUhandler,
		Parameters_Position_t *ptrPosHandler ,
		Parameters_Path_t *ptrPathHandler,
		double calib ,
		uint8_t *fAnglulo,
		uint8_t *fMeasurements,
		uint8_t *fcontrol,
		char buff[64],
		state_dir_t operation_mode_dir){

	////////////////////////////////////////BLOQUE DE MEDICION Y CONTROL//////////////////////////////////////////////////////


	//En este primera medicion se mide el el angulo actual del robot con respecto a una referencia.
	if (*fAnglulo){ // este se ejecutara cada periodo

		// Medimos el angulo actual
		getAngle(ptrMPUhandler, 0, calib, ptrPosHandler);
		// bajamos la bandera
		*fAnglulo = RESET;
	}
	// En la siguiente medicion medimos todos los parametros necesarios para el control posterior
	if (*fMeasurements){ // Este se ejecutara cada (periodo * 13 cuentas)

		// Medimos el angulo actual
		get_measuremets_parameters(ptrMotorhandler, ptrPosHandler, operation_mode_dir);
		// bajamos la bandera
		*fMeasurements = RESET;
	}

}


int extract_info ( command_t *cmd ,
				   char data[64],
				   unsigned char firstParameter[10],
				   unsigned char secondParameter[10],
				   unsigned char thirdParameter[10],
				   unsigned int *fparam,
				   unsigned int *sparam,
				   unsigned int *tparam){

	int counter = 0;
	uint8_t count_f = 0;
	uint8_t count_s = 0;
	uint8_t count_t = 0;

	uint8_t len_f = 0;
	uint8_t len_s = 0;
	uint8_t len_t = 0;




	while (1){

		while (cmd->payload[counter] != ' '){

			data[counter] = cmd->payload[counter];

			counter++;
		}

		// Le agregamos a data la terminacion nula
		data[counter] = '\0';

		// nos movemos a la siguiente posicion, ya que la posicion actual es un espacio
		counter++;

		if (cmd->payload[counter] == '#'){
			break;
		}

		count_f = counter;

		// Buscamos el firstParameter en el string
		while (cmd->payload[counter] != ' '){

			firstParameter[counter - count_f] = cmd->payload[counter];

			counter++;
		}

		len_f = counter - count_f; // Longitud del first parameter

		// Le agregamos a data la terminacion nula
		firstParameter[counter - count_f] = '\0';

		// nos movemos a la siguiente posicion, ya que la posicion actual es un espacio
		counter++;

		if (cmd->payload[counter] == '#'){
			break;
		}

		count_s = counter;

		// Buscamos el secondParameter en el string
		while (cmd->payload[counter] != ' '){

			secondParameter[counter - count_s] = cmd->payload[counter];

			counter++;
		}

		len_s = counter - count_s; // Longitud del second parameter

		// Le agregamos a data la terminacion nula
		secondParameter[counter - count_s] = '\0';

		// nos movemos a la siguiente posicion, ya que la posicion actual es un espacio
		counter++;

		if (cmd->payload[counter] == '#'){
			break;
		}

		count_t = counter;

		// Buscamos el thirdParameter en el string
		while (cmd->payload[counter] != ' '){

			thirdParameter[counter - count_t] = cmd->payload[counter];

			counter++;
		}

		len_t = counter - count_t; // Longitud del second parameter

		// Le agregamos a data la terminacion nula
		thirdParameter[counter - count_t] = '\0';

		// nos movemos a la siguiente posicion, ya que la posicion actual es un espacio
		counter++;

		if (cmd->payload[counter] == '#'){
			break;
		}
	}

	counter = 0;

	////////////////////////////////Bloque para convertir a valores enteros los first second y third parametros///////////////////

	// Comenzamos con el primer parametro

	if (len_f == 0){
		return 1; // Si se llega aca es porque no hay primer parametro, por loq ue no habra ni segundo ni tercero
	}

	// Si se llego aca es porque efectivamente hay algo en fistParameters que necesita ser convertido en un numero

	for (counter = len_f-1; counter > -1; counter--){

		*fparam += (firstParameter[abs(counter - (len_f-1))] -48) * pow(10,counter);
	}

	// CONSTRUIDO EL PRIMER PARAMETRO, se hace lo mismo para el segundo y el tercero

	// Comenzamos con el segundo parametro

	if (len_s == 0){
		return 2; // Si se llega aca es porque no hay segundo parametro, por lo que no habra tercer parametro
	}

	// Si se llego aca es porque efectivamente hay algo en secondParameter que necesita ser convertido en un numero

	for (counter = len_s-1; counter > -1; counter--){

		*sparam += (secondParameter[abs(counter - (len_s-1))]-48) * pow(10,counter);
	}

	// Comenzamos con el tercer parametro

	if (len_t == 0){
		return 3; // Si se llega aca es porque no hay tercer parametro, por lo que no habra tercer parametro
	}

	// Si se llego aca es porque efectivamente hay algo en thirdParameter que necesita ser convertido en un numero

	for (counter = len_t-1; counter > -1; counter--){

		*tparam += (thirdParameter[abs(counter - (len_t-1))] - 48) * pow(10,counter);
	}


	////////////// SI SE LLEGA HASTA ACA ES PORQUE YA TODO ESTA CONVERTIDO///////////////

	return 0;
}


void resetParameters(void){
	fparam = 0;
	sparam = 0;
	tparam = 0;
}




void led_state_callback (TimerHandle_t xTimer){

	GPIOxTooglePin(&handlerPinA5);

}
