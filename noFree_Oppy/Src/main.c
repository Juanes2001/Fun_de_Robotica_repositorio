 /**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include <stm32f4xx.h>
//
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>  // for usleep


#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "PwmDriver.h"
#include "USARTxDriver.h"
#include "RCCHunMHz.h"
#include "EXTIDriver.h"
#include "I2CDriver.h"
#include "MPUAccel.h"
#include "MotorsDriver.h"
#include "PosRobt.h"
#include "DMA.h"


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

// Estructura de estados
typedef enum{
	sLine = 0,
	sRoll
}state_t;

//DEFINICION DE FUNCIONES

void inSystem (void);
void parseCommands(char *stringVector);
float calibracionGyros (MPUAccel_Config *ptrMPUAccel, uint8_t axis);
void getAngle(MPUAccel_Config *ptrMPUAccel,float angle_init, double calibr,Parameters_Position_t *ptrParameter_position);
void On_motor_Straigh_Roll(Motor_Handler_t *ptrMotorhandler[2], uint8_t dir_s, uint8_t dir_r, state_t operation_mode);
void get_measuremets_parameters(BasicTimer_Handler_t *ptrTimer ,Motor_Handler_t *ptrMotorHandler[2], Parameters_Position_t *ptrParameter_position, state_t operation_mode);
void change_dir_straigh_Roll(Motor_Handler_t *ptrMotorhandler[2], uint8_t dir_r, uint8_t dir_s, state_t operation_mode);
void set_direction_straigh_roll (Motor_Handler_t *ptrMotorhandler[2], uint8_t dir_r, uint8_t dir_s, state_t operation_mode);
void stop (Motor_Handler_t *ptrMotorhandler[2]);

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
char cmd[32];
unsigned int firstParameter  = 0;
unsigned int secondParameter = 0;
unsigned int thirdParameter  = 0;
char userMsg[64];

//////Banderas y estados-----------
state_t Mode              = sLine;
uint8_t flag_action       = 0;
uint8_t flag_angulo       = 0;
uint8_t flag_measurements = 0;
uint8_t flag_Go_Straigh   = 0;
uint8_t flag_GoTo_Straigh = 0;


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

// VARIABLES VARIAS DEL ROBOT
#define fixed_dutty 28 // Fixed dutty cycle, velocidad constante
#define fixed_sample_period 16 // Periodo en milisegundos de muestreo de datos de encoder

int main(void)
{

	//Activamos el FPU o la unidad de punto flotante
 	SCB -> CPACR |= (0xF << 20);
	RCC_enableMaxFrequencies(RCC_100MHz);
	inSystem ();

	//Calculamos el setpoint en la que queremos que el robot controle la velocidad de cada motor
	velSetPoint = (0.00169*fixed_dutty + 0.0619);

//	On_motor_Straigh_Roll(handler_Motor_Array, *((uint8_t *) NULL), SET, sLine);

	// calibramos el Giroscopio para que tengamos una medida de error controlable
//	cal_Gyro = calibracionGyros(&handler_MPUAccel_6050, CALIB_Z);

    /* Loop forever */
	while(1){

		if (rxData != '\0'){
			writeChar(&handlerUSART, rxData);
			bufferReception[counterReception] = rxData;
			counterReception++;

			if (rxData == '@'){
				doneTransaction = SET;

				bufferReception[counterReception-1] = '\0';

				counterReception = 0;

			}else if (rxData == 'z'){

				memset(bufferReception, 0, sizeof(bufferReception));
				counterReception = 0;
				writeMsg(&handlerUSART, "Buffer Vaciado\n \r");
			}

				rxData = '\0';

		}





		////////////////////////////////////////BLOQUE DE MEDICION Y CONTROL//////////////////////////////////////////////////////


		//En este primera medicion se mide el el angulo actual del robot con respecto a una referencia.
		if (flag_angulo){

			// Medimos el angulo actual
			getAngle(&handler_MPUAccel_6050, 0, cal_Gyro, &parameters_Pos_Robot);
			// bajamos la bandera
			flag_angulo = RESET;
		}
		// En la siguiente medicion medimos todos los parametros necesarios para el control posterior
		if (flag_measurements){

			// Medimos el angulo actual
			get_measuremets_parameters(&handlerTIM2_PARAMETROS_MOVIMIENTO, handler_Motor_Array, &parameters_Pos_Robot, Mode);
			// bajamos la bandera
			flag_measurements = RESET;
		}


		// En esta parte ya usamos las medidas halladas para mover el robot en linea recta dependiendo de la operacion y el comando deseado
		if (flag_Go_Straigh){

			// Si estamos aqui se quiere solo que el robot vaya hacia adelante y el linea recta

		}





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

	handlerTimerBlinky.ptrTIMx                           = TIM3;
	handlerTimerBlinky.TIMx_Config.TIMx_interruptEnable  = BTIMER_ENABLE_INTERRUPT;
	handlerTimerBlinky.TIMx_Config.TIMx_mode             = BTIMER_MODE_UP;
	handlerTimerBlinky.TIMx_Config.TIMx_speed            = BTIMER_SPEED_100MHz_100us;
	handlerTimerBlinky.TIMx_Config.TIMx_period           = 500;
	BasicTimer_Config(&handlerTimerBlinky);
	startTimer(&handlerTimerBlinky);


	// DEFINICION DEL TIM4 PARA DELAY
	inTIM4();


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
	handlerPWM_1.config.duttyCicle  = 0;
//	counter = 50;
	handlerPWM_1.config.periodo     = 40; // se maneja 25 hz por testeo
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
	handlerPWM_2.config.duttyCicle  = 0;
	handlerPWM_2.config.periodo     = 40;// se maneja 25 hz por testeo
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
	GPIO_WritePin_Afopt(&handlerIn1PinC12, RESET); // default

	handlerIn2PinD2.pGPIOx                             = GPIOD;
	handlerIn2PinD2.GPIO_PinConfig.GPIO_PinAltFunMode  = AF0;
	handlerIn2PinD2.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
	handlerIn2PinD2.GPIO_PinConfig.GPIO_PinNumber      = PIN_2;
	handlerIn2PinD2.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerIn2PinD2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerIn2PinD2.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerIn2PinD2);
	GPIO_WritePin_Afopt(&handlerIn2PinD2, RESET); // default


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



	//////////////////////////////////////////////////// /////////////////// //////////////////////////////////////////////

	///////////////////////////////////////////Comunicación serial para comandos //////////////////////////////////////////////


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

	//////////////////////////////////////////////////// /////////////////// //////////////////////////////////////////////

	///////////////////////////////////////////Timer para el control de la velocidad//////////////////////////////////////////////

	handlerTIM2_PARAMETROS_MOVIMIENTO.ptrTIMx                           = TIM2;
	handlerTIM2_PARAMETROS_MOVIMIENTO.TIMx_Config.TIMx_interruptEnable  = BTIMER_ENABLE_INTERRUPT;
	handlerTIM2_PARAMETROS_MOVIMIENTO.TIMx_Config.TIMx_mode             = BTIMER_MODE_UP;
	handlerTIM2_PARAMETROS_MOVIMIENTO.TIMx_Config.TIMx_speed            = BTIMER_SPEED_100MHz_10us;
	handlerTIM2_PARAMETROS_MOVIMIENTO.TIMx_Config.TIMx_period           = fixed_sample_period;
	BasicTimer_Config(&handlerTIM2_PARAMETROS_MOVIMIENTO);


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
	handler_PINB8_I2C1.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_HIGH;

	handler_PINB9_I2C1.pGPIOx                             = GPIOB;
	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinAltFunMode  = AF4;
	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinNumber      = PIN_9;
	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_OPENDRAIN;
	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_HIGH;

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


void parseCommands(char *stringVector){

	sscanf(stringVector, "%s %u %u %u %s", cmd ,&firstParameter, &secondParameter, &thirdParameter, userMsg);




	if (strcmp(cmd, "help") == 0){

		writeMsg(&handlerUSART, "HELP MENU CMD : \n");
		writeMsg(&handlerUSART, "1)  go #dir 1--> Ahead , 0-->back Ward\n");
		writeMsg(&handlerUSART, "2)  goto #dir #Distance \n");
		writeMsg(&handlerUSART, " \n");

	}else if (strcmp(cmd, "go") == 0){
		NULL;
		// Si llegamos a este comando, lo que se quiere es ir en linea recta usando un control PID
		On_motor_Straigh_Roll(handler_Motor_Array,*((uint8_t *) NULL), firstParameter, Mode); // Encendemos los motores para irnos hacia adelante y con una velocidad fija
		startTimer(&handlerTIM2_PARAMETROS_MOVIMIENTO); // Comenzamos el muestreo de datos con los que aplicaremos un control adecuado
		flag_Go_Straigh = SET;

	}else if (strcmp(cmd, "goto") == 0){
		// Si estamos aqui es porque se quiere ir recorriendo una distancia especifica
		On_motor_Straigh_Roll(handler_Motor_Array,*((uint8_t *) NULL), firstParameter, Mode); // Encendemos los motores para irnos hacia adelante y con una velocidad fija
		startTimer(&handlerTIM2_PARAMETROS_MOVIMIENTO); // Comenzamos el muestreo de datos con los que aplicaremos un control adecuado
		parameters_Path_Robot.line_Distance = secondParameter; // almacenamos la distancia en milimrtros a recorrer
		flag_GoTo_Straigh = SET;

	}else if (strcmp(cmd, "roll") == 0){
		// si estamos aqui, este comando lo que hara es girar el robot indefinidamente

	}




	else if (strcmp(cmd, "stop") == 0){
		 // Este comando lo que busca es apagar el robot y detenerlo de su estado de movimiento
		stop(handler_Motor_Array); // Apagamos los motores
		stopTimer(&handlerTIM2_PARAMETROS_MOVIMIENTO); // Detenemos los muestreos
	}


	else {
		writeMsg(&handlerUSART, "Comando Incorrecto :c \n");


	}


}


// Interrupcion usart 1
void usart2Rx_Callback(void){

	rxData = getRxData();

}


//Interrupción Timer 3
void BasicTimer3_Callback(void){
	GPIOxTooglePin(&handlerPinA5);
}

//Interrupcion Timer 2

void BasicTimer2_Callback(void){


	// Levantamos bandera que calcula el angulo actual
	flag_angulo = SET;

	// Levantamos la bandera que corresponde con los calculos  odometricos del robot, como la distancia
	// Recorrida, la posicion actual y la velocidad

	flag_measurements = SET;

	// EN EL MAIN ESTAS DOS BANDERAS SE ANALIZARAN Y SE EJECUTARAN
}


//Interrupciones de Exti
void callback_extInt1(void){


}

void callback_extInt3(void){


}


//Definicion de funciones varias


// Calibracion Gyros:

float calibracionGyros (MPUAccel_Config *ptrMPUAccel, uint8_t axis){

	uint16_t  numMedidas = 50;
	float    medidas    = 0;
	float    suma       = 0;
	uint8_t  contador   = 0;
	float    promedio   = 0;

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
}

void get_measuremets_parameters(BasicTimer_Handler_t *ptrTimer ,Motor_Handler_t *ptrMotorHandler[2], Parameters_Position_t *ptrParameter_position, state_t operation_mode){

	//Verificamos el modo
	if(operation_mode == sLine){ // Levantamos la vandera que calcula todos los parametros necesarios para el control



		//Acumulamos los angulos
		sum_ang += ptrParameter_position->grad_relativo;
		//Se acumula el tiempo
		time_accumulated += ptrTimer->TIMx_Config.TIMx_period;

		//----------------Accion a realizar con un tiempo especifico--------------------
		if(counting_action >= timeAction_TIMER_Sampling)
		{
			//Guardamos el tiempo entre acciones especificas
			time_accion = time_accumulated;
			//Calculamos el angulo promedio y la establecemos como el angulo relativo
			promAng = sum_ang / counting_action;
			ptrParameter_position->phi_relativo = (promAng * M_PI) / 180;          //[rad]
			// Con la siguiente accion lo que hacemos es conseguir un angulo con mucha mas precision decimal debido a la funcion atan2
			ptrParameter_position->phi_relativo = atan2(sin(ptrParameter_position->phi_relativo),cos(ptrParameter_position->phi_relativo));
			//Calculamos la velocidad
			ptrMotorHandler[0]->parametersMotor.dis = (paso_mm_1 * ptrMotorHandler[0]->parametersMotor.counts);      //[mm]
			ptrMotorHandler[1]->parametersMotor.dis = (paso_mm_2 * ptrMotorHandler[1]->parametersMotor.counts);      //[mm]
			ptrMotorHandler[0]->parametersMotor.vel = ptrMotorHandler[0]->parametersMotor.dis / time_accion;      //[m/s]
			ptrMotorHandler[1]->parametersMotor.vel = ptrMotorHandler[1]->parametersMotor.dis / time_accion;      //[m/s]
			//Reiniciamos el numero de conteos
			ptrMotorHandler[0]->parametersMotor.counts = 0;
			ptrMotorHandler[1]->parametersMotor.counts = 0;
			//Reiniciamos variable
			sum_ang = 0;
			//Reiniciamos tiempo
			time_accumulated = 0;
			//Reiniciamos el contador de accion
			counting_action = 0;
			//Levantamos bandera
			flag_action = 1;
		}
		else{ counting_action++; }
	}
	else if(Mode == sRoll)
	{
		//----------------Accion a realizar con un tiempo especifico--------------------
		if(counting_action>=timeAction_TIMER_Sampling)
		{
			//Guardamos el tiempo entre acciones especificas
			time_accion = time_accumulated;
			//Calculo de la distancia recorrida por cada rueda
			ptrMotorHandler[0]->parametersMotor.dis = (paso_mm_1 * ptrMotorHandler[0]->parametersMotor.counts);// Calculamos la distancia recorrida contando cuantos pasos a dado la rueda izquierda //[mm]
			ptrMotorHandler[1]->parametersMotor.dis = (paso_mm_2 * ptrMotorHandler[1]->parametersMotor.counts);// Calculamos la distancia recorrida contando cuantos pasos a dado la rueda derecha   //[mm]
			ptrMotorHandler[0]->parametersMotor.vel = ptrMotorHandler[0]->parametersMotor.dis / time_accion; // Calculamos la velocidad de la rueda izquierda     //[m/s]
			ptrMotorHandler[1]->parametersMotor.vel = ptrMotorHandler[1]->parametersMotor.dis / time_accion; // Calculamos la velocidad de la rueda derecha    //[m/s]
			//Reiniciamos el numero de conteos
			ptrMotorHandler[0]->parametersMotor.counts = 0;
			ptrMotorHandler[1]->parametersMotor.counts = 0; // RESETEAMOS LAS CUENTAS

			//Cálculo ángulo debido al desplazamiento del ICR
			ang_for_Displament += (((ptrMotorHandler[1]->parametersMotor.dis - ptrMotorHandler[0]->parametersMotor.dis) * 100)/ distanceBetweenWheels)*(180/M_PI); //[°]
			//Reiniciamos tiempo
			time_accumulated = 0;
			//Reiniciamos el contador de acción
			counting_action  = 0;
		}
		else{counting_action++;}

		//Combinar ambos ángulos
		ang_complementary = ptrParameter_position->grad_relativo + ang_for_Displament;
	}
	else{  __NOP(); }

}

void On_motor_Straigh_Roll(Motor_Handler_t *ptrMotorhandler[2], uint8_t dir_s, uint8_t dir_r, state_t operation_mode){


	if (operation_mode == sLine){
				//Activamos el motor
				// ENCENCEMOS EL MOTOR 1 (LEFT)
					// Seteamos correctamente la direccion de cada motor
					set_direction_straigh_roll(ptrMotorhandler, dir_r, dir_s, operation_mode);

					enableOutput(ptrMotorhandler[0]->phandlerPWM);
					GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOEN,SET); // Encendemos el motor 1

					// ENCENCEMOS EL MOTOR 2 (Right)
					//Se enciende el motor 2
					enableOutput(ptrMotorhandler[1]->phandlerPWM);
					GPIO_WritePin_Afopt (ptrMotorhandler[1]->phandlerGPIOEN,SET);


	}else if (operation_mode == sRoll){
				//Activamos el motor
				// ENCENCEMOS EL MOTOR 1 (LEFT)
					set_direction_straigh_roll(ptrMotorhandler, dir_r, dir_s, operation_mode);

					//Se enciende el motor 1
					enableOutput(ptrMotorhandler[0]->phandlerPWM);
					GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOEN,SET); // Encendemos el motor 1

					// ENCENCEMOS EL MOTOR 2 (Right)
					//Se enciende el motor 2
					enableOutput(ptrMotorhandler[1]->phandlerPWM);
					GPIO_WritePin_Afopt (ptrMotorhandler[1]->phandlerGPIOEN,SET);


	}

}

void set_direction_straigh_roll (Motor_Handler_t *ptrMotorhandler[2], uint8_t dir_r, uint8_t dir_s, state_t operation_mode){

	// Esta funcion setea correctamente la direccion de los motores dependiendo de lo que se quiera.
	if (operation_mode == sLine){
		// Si estamos aqui es porque queremos Setear la direccion hacia adelante o hacia atras

		// Si queremos ir hacia adelante

		// Primero revisamos en que direccion se encuentra el robot para ver si si se aplica
		// el cambio o no
		if ((ptrMotorhandler[0]->configMotor.dir != dir_s)){
			// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
			GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOIN, dir_s & SET); // La direccion estaba en RESET, la cambiamos a SET
			PWMx_Toggle(ptrMotorhandler[0]->phandlerPWM);

		}

		if ((ptrMotorhandler[1]->configMotor.dir != dir_s)){
			// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
			GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN, dir_s & SET); // La direccion estaba en RESET, la cambiamos a SET
			PWMx_Toggle(ptrMotorhandler[1]->phandlerPWM);
		}
		// Puede que no analice ningun if y simplemente no haga nada

	}else if (operation_mode == sRoll){
		// si estamos aca es porque queremos cambiar la direccion de rotacion


		if (dir_r == 0){
			// Si queremos ir en centido CW o horario

			// Primero revisamos en que direccion se encuentra el robot para ver si si se aplica
			// el cambio o no
			if ((ptrMotorhandler[0]->configMotor.dir != !dir_r)){
				// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
				GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOIN, !dir_r & SET); // La direccion estaba en RESET, la cambiamos a SET
				PWMx_Toggle(ptrMotorhandler[0]->phandlerPWM);

			}

			if ((ptrMotorhandler[1]->configMotor.dir != dir_r)){
				// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
				GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN, dir_r & SET); // La direccion estaba en SET, la cambiamos a RESET
				PWMx_Toggle(ptrMotorhandler[1]->phandlerPWM);
			}
			// Puede que no analice ningun if y simplemente no haga nada


		}else{
			// Si queremos ir en centido CCW o anti horario

			// Primero revisamos en que direccion se encuentra el robot para ver si si se aplica
			// el cambio o no
			if ((ptrMotorhandler[0]->configMotor.dir != !dir_r)){
				// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
				GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOIN, !dir_r & SET); // La direccion estaba en SET, la cambiamos a RESET
				PWMx_Toggle(ptrMotorhandler[0]->phandlerPWM);

			}

			if ((ptrMotorhandler[1]->configMotor.dir != dir_r)){
				// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
				GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN, dir_r & SET); // La direccion estaba en RESET, la cambiamos a SET
				PWMx_Toggle(ptrMotorhandler[1]->phandlerPWM);
			}
			// Puede que no analice ningun if y simplemente no haga nada


		}

	}
}

void change_dir_straigh_roll(Motor_Handler_t *ptrMotorhandler[2], uint8_t dir_s,uint8_t dir_r, state_t operation_mode){

	if (operation_mode == sLine){
		// Si estamos aqui es porque queremos cambiar la direccion en linea recta correctamente

		// antes de cambiar la direccion apagamos los motores
		GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOEN,RESET);
		GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN,RESET);

		// Primero revisamos en que direccion se encuentra el robot para ver si si se aplica
		// el cambio o no
		if ((ptrMotorhandler[0]->configMotor.dir != dir_s)){
			// si estamos aqui es porque se quiere cambiar la direccion del robot

			// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
			GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOIN, dir_s & SET); // La direccion estaba en RESET, la cambiamos a SET
			PWMx_Toggle(ptrMotorhandler[0]->phandlerPWM);

		}

		if ((ptrMotorhandler[1]->configMotor.dir != dir_s)){
			// si estamos aqui es porque se quiere cambiar la direccion del robot

			// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
			GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN, dir_s & SET); // La direccion estaba en RESET, la cambiamos a SET
			PWMx_Toggle(ptrMotorhandler[1]->phandlerPWM);
		}
		// Puede que no analice ningún if y simplemente no haga nada


		// volvemos a encender los motores
		GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOEN,SET);
		GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN,SET);





	}else if (operation_mode == sRoll){
		// si estamos aca es porque queremos cambiar la direccion de rotacion


		if (dir_r == 0){
			// Si queremos ir en centido CW o horario

			// Primero revisamos en que direccion se encuentra el robot para ver si si se aplica
			// el cambio o no
			// si estamos aqui es porque se quiere cambiar la direccion DEL ROBOT A CW

			// antes de cambiar la direccion apagamos los motores
			GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOEN,RESET);
			GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN,RESET);

			if ((ptrMotorhandler[0]->configMotor.dir != !dir_r)){
				// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
				GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOIN, !dir_r & SET); // La direccion estaba en RESET, la cambiamos a SET
				PWMx_Toggle(ptrMotorhandler[0]->phandlerPWM);

			}

			if ((ptrMotorhandler[1]->configMotor.dir != dir_r)){
				// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
				GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN, dir_r & SET); // La direccion estaba en SET, la cambiamos a RESET
				PWMx_Toggle(ptrMotorhandler[1]->phandlerPWM);
			}
			// Puede que no analice ningun if y simplemente no haga nada


			// volvemos a encender los motores
			GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOEN,SET);
			GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN,SET);



		}else{
			// Si queremos ir en centido CCW o anti horario

			// Primero revisamos en que direccion se encuentra el robot para ver si si se aplica
			// el cambio o no
			// si estamos aqui es porque se quiere cambiar la direccion del robot

			// antes de cambiar la direccion apagamos los motores
			GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOEN,RESET);
			GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN,RESET);

			if ((ptrMotorhandler[0]->configMotor.dir != !dir_r)){
				// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
				GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOIN, !dir_r & SET); // La direccion estaba en SET, la cambiamos a RESET
				PWMx_Toggle(ptrMotorhandler[0]->phandlerPWM);

			}

			if ((ptrMotorhandler[1]->configMotor.dir != dir_r)){
				// cambiamos la direccion cambiando los pines in pero tambien aplicando un toogle al PWM en cada caso
				GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN, dir_r & SET); // La direccion estaba en RESET, la cambiamos a SET
				PWMx_Toggle(ptrMotorhandler[1]->phandlerPWM);
			}
			// Puede que no analice ningun if y simplemente no haga nada

			// volvemos a encender los motores
			GPIO_WritePin_Afopt(ptrMotorhandler[0]->phandlerGPIOEN,SET);
			GPIO_WritePin_Afopt(ptrMotorhandler[1]->phandlerGPIOEN,SET);

		}
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
