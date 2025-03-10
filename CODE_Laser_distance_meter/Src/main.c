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
#include "stm32f4xx.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>  // for usleep


#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "USARTxDriver.h"
#include "RCCHunMHz.h"
#include "PwmDriver.h"
#include "I2CDriver.h"
#include "AdcDriver.h"

#define C0 299792458

void inSystem (void);
void parseCommands(char *stringVector);
void builtTerminalString (char** terminalString);
float counts2time(uint64_t counts);



//Definición Handlers
//GPIO
//Pin del User blinky
GPIO_Handler_t handlerPinA5         = {0};

//Pines de comunicacion USART
GPIO_Handler_t handlerPinRx         = {0};
GPIO_Handler_t handlerPinTx         = {0};

// Pin de salida PWM
GPIO_Handler_t handlerPin_one_pulse 	= {0};
GPIO_Handler_t handlerPin_PWM_out 		= {0};
GPIO_Handler_t handlerPin_ADC_Vx 		= {0};
GPIO_Handler_t handlerPin_ADC_Vy 		= {0};


//Pin para visualizar la velocidad del micro
GPIO_Handler_t handlerMCO2Show      = {0};

// Pines para I2C1
GPIO_Handler_t handler_PINB8_I2C1   = {0};
GPIO_Handler_t handler_PINB9_I2C1   = {0};

//Timers
BasicTimer_Handler_t handlerTimerBlinky = {0}; // Timer 3
BasicTimer_Handler_t handlerADCTim = {0}; // Timer 4 ADC
BasicTimer_Handler_t handlerTim1_Conteo = {0}; // Timer1, para conteo a alta resolucion

//PWM
PWM_Handler_t handlerPWM_one_pulse   = {0};

//I2C
I2C_Handler_t handler_I2C1 = {0};

//handler para ADC
ADC_Config_t handlerADC = {0};

//Usart
USART_Handler_t handlerUSART ={0};

// Variables para los comandos
char bufferReception[64];
uint8_t counterReception = 0;
uint8_t rxData = '\0';
char cmd[32];
unsigned int firstParameter;
unsigned int secondParameter;
unsigned int thirdParameter;
char userMsg[64];
char bufferMsg[64];

float t_mosfet = 10; // Tiempo en de delay en el que se demora el mosfet en prenderse
float t_lockin = 30; // Tiempo en el que se demora la señal en ser procesada en el lock in, esto
					 // incluye el tiemoo de demora de la deteccion, el tiempo de procesamiento de la señal
					 // dentro del lock in hasta que sea detectado por el ADC.

// ADC variables
uint32_t adcData[2] ;
uint8_t counterADC = 0;

// Banderas
uint8_t adcFlag 		= RESET;
uint8_t doneTransaction = RESET;
uint8_t stopTimerFlag	= RESET;
uint8_t flagMeas 		= RESET;

//Mensajes
const char* msg_NotWorking = "\n--------Astar isn't working properly----------\n";
const char* msg_InsertGrid = "\n------------Insert the char grid--------------\n";

// OTRAS VARIABLES
uint8_t dutty_cycle 		= 50;
uint64_t cuentas_totales    = 0;
double t_total 				= 0;
double threshold    		= 0;
float voltageX 				= 0;
float voltageY 				= 0;
float voltageT				= 0;
float vref 					= 0;
int counter 				= -1;
double distancia 			= 0;



int main(void)
{

	//Activamos el FPU o la unidad de punto flotante
 	SCB -> CPACR |= (0xF << 20);



	inSystem();


    /* Loop forever */
	while(1){

		// --COMENZAMOS LA MEDICION MIDIENDO EL MOMENTO EN EL QUE SE ENCIENDE EL LASER USANDO EL TIEMPO
		//   t0 = tdelay,  tdelay_mosfet_o_bjt (no hay necesidad de ADC o ningun fotodiodo a la entrada por
		//   facilidad)




		if (rxData == 'm'){
			if (~flagMeas){

				flagMeas = enableOutput(&handlerPWM_one_pulse);

				startConvertion();
			}else{

				flagMeas = disableOutput(&handlerPWM_one_pulse);

			}

			rxData = '\0';
		}

			// Aqui colocamos la funcion de comenzar pulso de tal forma que se pueda sincronizar
			// el tiempo de comienzo con el tiempo de finalizacion del proceso




		if (adcFlag){

			voltageX = adcData[0]*(3.3/(powf(2.0,12)));
//			voltageY = adcData[1]*(3.3/(powf(2.0,12)));

//			voltageT = sqrt(pow(voltageX,2)+pow(voltageY,2));

//			sprintf(userMsg,
//					"%.3f\t%.2f\t%.3f\t%.2f\t%.3f\n",
//					voltageX,
//					1.65,
//					voltageY,
//					1.65,
//					voltageT);

			sprintf(userMsg,
					"%.3f\n",
					voltageX);

			writeMsg(&handlerUSART, userMsg);

			adcFlag = RESET;
			adc_CONT_ON();
			startConvertion();
		}





		// --Se medira el pulso de luz usando un timer de tal manera que podamos encender y apagar en el proceso cada vez que un pulso
		//   de luz sea enviado por lo que es necesario un conteo rapido y preciso del tiempo que transcurre midiendo la cantidad de cuentas
		//   que el timer ha contado desde el momento cero, por lo que tdelay_mosfet_o_bjt debe ser un parametro constante que se debe de pasar a cuentas
		//	 para luego ser tomado en cuenta en el calculo del tiempo del recorrido.

		// --Luego el pulso de luz recorre la distancia y regresa y entra al circuito amplificador lock in
		//   en el cual es necesario medir los tiempos que tarda el lockin en lockear el ruido y extraer la señal
		//   que se requiere, este tiempo en total seria t_lockin el cual es un parametro fijo constante el cual se transforma
		//   en cuentas del timer par aluego hacer la conversion mas facilmente.


		// --Luego de extraida la señal del ruido de fondo, el ADC esta continuamente midiendo, por lo que es necesario
		//   hacer pruebas primero para saber como diferenciar entre ruido de fondo(lo que entrega a la salida el lock in
		//	 Vx y Vy que luego la señal de voltaje total será Vtotal = sqrt(Vx^2+Vy^2). )

		// --Si si existen diferencias gracias a las pruebas, entonces al ser detectada Vtotal (Puede ser detectada)
		//	 usando depronto un threshold, pero solo puede lograrse gracias a pruebas)
		//	 entonces se para inmediatamente el timer y se toman las cuentas totales medidas y se hace la resta y la conversion.
		//   Este sera entonces el tiempo total medido desde el momento en el que se enciende el mosfet hasta el momento
		//   en el que se cierra la medicion con el ADC.

		// --Finalmente teniendo las cuentas totales medidas, se hace la conversion entre cuentas y tiempo en segundos, y luego se mide
		//   la distancia usando la formula de velocidad = distancia/tiempo
		//   distancia = C*(t_total-t0-t_lockin)/2.






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


	}// FIN DEL LOOP
}


void inSystem (void){


	// Activamos la maxima velocidad del microcontrolador
	RCC_enableMaxFrequencies(RCC_100MHz);
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
	GPIO_WritePin(&handlerPinA5, RESET);

	handlerTimerBlinky.ptrTIMx                           = TIM5;
	handlerTimerBlinky.TIMx_Config.TIMx_interruptEnable  = BTIMER_ENABLE_INTERRUPT;
	handlerTimerBlinky.TIMx_Config.TIMx_mode             = BTIMER_MODE_UP;
	handlerTimerBlinky.TIMx_Config.TIMx_speed            = BTIMER_SPEED_100MHz_100us;
	handlerTimerBlinky.TIMx_Config.TIMx_period           = 1000;
	BasicTimer_Config(&handlerTimerBlinky);



	//////////////////////////////////////////////////// /////////////////// //////////////////////////////////////////////

	///////////////////////////////////////////Comunicación serial para comandos //////////////////////////////////////////////


	//Comunicacion serial

	handlerPinRx.pGPIOx                             = GPIOA;
	handlerPinRx.GPIO_PinConfig.GPIO_PinAltFunMode  = AF7;
	handlerPinRx.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerPinRx.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerPinRx.GPIO_PinConfig.GPIO_PinNumber      = PIN_3;
	handlerPinRx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinRx.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_HIGH;
	GPIO_Config(&handlerPinRx);

	handlerPinTx.pGPIOx                             = GPIOA;
	handlerPinTx.GPIO_PinConfig.GPIO_PinAltFunMode  = AF7;
	handlerPinTx.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerPinTx.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerPinTx.GPIO_PinConfig.GPIO_PinNumber      = PIN_2;
	handlerPinTx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinTx.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_HIGH;
	GPIO_Config(&handlerPinTx);

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




	// Conversión
//	handlerADC.channelVector[0] 	= ADC_CHANNEL_0;
//	handlerADC.channelVector[1] 	= ADC_CHANNEL_1;
	handlerADC.channel				= ADC_CHANNEL_0;
	handlerADC.dataAlignment 		= ADC_ALIGNMENT_RIGHT;
	handlerADC.resolution			= ADC_RESOLUTION_12_BIT;
	handlerADC.samplingPeriod 		= ADC_SAMPLING_PERIOD_3_CYCLES;
	handlerADC.continuosModeEnable  = ADC_CONT_ENABLE;
	handlerADC.multiChannel 		= ADC_MULTCH_DISABLE;
	handlerADC.watchdogs_Enable 	= ADC_WATCHDOG_DISABLE;
	handlerADC.threshold_up 		= pow(2.0,12)/2;
	handlerADC.threshold_down 		= 0;
	adc_Config(&handlerADC);

	handlerPin_ADC_Vx.pGPIOx = GPIOA;
	handlerPin_ADC_Vx.GPIO_PinConfig.GPIO_PinAltFunMode = AF0;
	handlerPin_ADC_Vx.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	handlerPin_ADC_Vx.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerPin_ADC_Vx.GPIO_PinConfig.GPIO_PinNumber = PIN_0;
	handlerPin_ADC_Vx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPin_ADC_Vx.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPin_ADC_Vx);

	handlerPin_ADC_Vy.pGPIOx = GPIOA;
	handlerPin_ADC_Vy.GPIO_PinConfig.GPIO_PinAltFunMode = AF0;
	handlerPin_ADC_Vy.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	handlerPin_ADC_Vy.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerPin_ADC_Vy.GPIO_PinConfig.GPIO_PinNumber = PIN_7;
	handlerPin_ADC_Vy.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPin_ADC_Vy.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPin_ADC_Vy);



	// TIMER 1 PARA CONTEO DE TIEMPO DEL PROCESO, ALTA RESOLUCION DE CONTEO
	handlerTim1_Conteo.ptrTIMx = TIM1;
	handlerTim1_Conteo.TIMx_Config.TIMx_interruptEnable = BTIMER_DISABLE_INTERRUPT;
	handlerTim1_Conteo.TIMx_Config.TIMx_mode = BTIMER_MODE_UP;
	handlerTim1_Conteo.TIMx_Config.TIMx_period = 100;
	handlerTim1_Conteo.TIMx_Config.TIMx_speed = BTIMER_SPEED_100MHz_10ns;
	BasicTimer_Config(&handlerTim1_Conteo);



	// PWM definicion y PIN USANDO EL MODO DE SIMGLE PULSE
	handlerPWM_one_pulse.ptrTIMx            = TIM2;
	handlerPWM_one_pulse.config.channel     = PWM_CHANNEL_2;
	handlerPWM_one_pulse.config.duttyCicle  = dutty_cycle;
	handlerPWM_one_pulse.config.periodo     = 1000;
	handlerPWM_one_pulse.config.prescaler   = PWM_SPEED_100MHz_1us;
	handlerPWM_one_pulse.config.polarity    = PWM_DISABLE_POLARITY;
	handlerPWM_one_pulse.config.optocoupler = PWM_DISABLE_OPTOCOUPLER;
	handlerPWM_one_pulse.config.one_pulse   = PWM_DISABLE_ONE_PULSE;
	pwm_Config(&handlerPWM_one_pulse);
	startPwmSignal(&handlerPWM_one_pulse);

	handlerPin_PWM_out.pGPIOx                             = GPIOB;
	handlerPin_PWM_out.GPIO_PinConfig.GPIO_PinAltFunMode  = AF1;
	handlerPin_PWM_out.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerPin_PWM_out.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerPin_PWM_out.GPIO_PinConfig.GPIO_PinNumber      = PIN_3;
	handlerPin_PWM_out.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPin_PWM_out.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPin_PWM_out);


//	////////////////////////////////Configuracion PINES B8 (SCL) B9 (SDA) e I2C1 //////////////////////////////////////////////
//
//
//	handler_PINB8_I2C1.pGPIOx                             = GPIOB;
//	handler_PINB8_I2C1.GPIO_PinConfig.GPIO_PinAltFunMode  = AF4;
//	handler_PINB8_I2C1.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
//	handler_PINB8_I2C1.GPIO_PinConfig.GPIO_PinNumber      = PIN_8;
//	handler_PINB8_I2C1.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_OPENDRAIN;
//	handler_PINB8_I2C1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
//	handler_PINB8_I2C1.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
//	GPIO_Config(&handler_PINB8_I2C1);
//
//	handler_PINB9_I2C1.pGPIOx                             = GPIOB;
//	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinAltFunMode  = AF4;
//	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
//	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinNumber      = PIN_9;
//	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_OPENDRAIN;
//	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
//	handler_PINB9_I2C1.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
//	GPIO_Config(&handler_PINB9_I2C1);
//
//	handler_I2C1.ptrI2Cx = I2C1;
//	handler_I2C1.I2C_Config.clkSpeed = MAIN_CLOCK_50_MHz_FOR_I2C;
//	handler_I2C1.I2C_Config.slaveAddress = 0;
//	handler_I2C1.I2C_Config.modeI2C = I2C_MODE_FM;
//	i2c_config(&handler_I2C1);

}


void parseCommands(char *stringVector){

	sscanf(stringVector, "%s %u %u %u %s", cmd ,&firstParameter, &secondParameter, &thirdParameter, userMsg);


	if (strcmp(cmd, "help") == 0){

		writeMsg(&handlerUSART, "HELP MENU CMD : \n");

	}



}



// Interrupcion usart 1
void usart2Rx_Callback(void){

	rxData = getRxData();

}



//Callback para interrupciones posterior a la multiconversion
void adcComplete_Callback(void){
//	counter++;
//	if (counter % 2 == 0){
//		adcData[0] = getADC();
//	}else if (counter % 2 != 0){
//		adcData[1] = getADC();
//		adc_CONT_OFF();
//		adcFlag = SET;
//		counter = -1;
//	}

	adcData[0] = getADC();
	adcFlag = SET;
	adc_CONT_OFF();
}

//void watchdogs_Callback(void){
//
//	stopTimerFlag = SET;
//	adc_OFF();
//
//}

//Interrupción Timer 3
void BasicTimer5_Callback(void){

	GPIOxTooglePin(&handlerPinA5);

}

float counts2time(uint64_t counts){
	// Usando la velocidad mas rápida del micro y 20ns por cuenta tenemos que
	float time = 0;

	time = 20.0 * counts;

	return time;
}




