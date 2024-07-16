/**
 ******************************************************************************
 * Solucion Tarea 5
 * Autor: Juan Esteban Rodriguez Ochoa
 * Correo: jrodriguezoc@unal.edu.co
 ******************************************************************************
 ******************************************************************************
 */
#include "BasicTimer.h"
#include "GPIOxDriver.h"
#include "USARTxDriver.h"
#include "EXTIDriver.h"
#include "PwmDriver.h"
#include "I2CDriver.h"
#include "AdcDriver.h"
#include "OLEDDriver.h"
#include "RTCDriver.h"
#include "FPUDriver.h"
#include "PIDDriver.h"
#include "MPUAccel.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


//Pines controladores para el motor

GPIO_Handler_t handlerLEDPin = {0};
GPIO_Handler_t handlerPWMPinServo = {0};
GPIO_Handler_t handlerPWMPinMotor = {0};
GPIO_Handler_t handlerI2cSCL     = {0};
GPIO_Handler_t handlerI2cSDA    = {0};
GPIO_Handler_t handlerExInSWPin = {0};
GPIO_Handler_t handlerPinRele1 = {0};
GPIO_Handler_t handlerPinRele2 = {0};

ADC_Config_t handlerADCJoy = {0};


GPIO_Handler_t handlerUSARTPinTx       = {0};
GPIO_Handler_t handlerUSARTPinRx       = {0};

//Timer para controlador de velocidades
BasicTimer_Handler_t handlerBlinky = {0};
BasicTimer_Handler_t handlerTimADCConver = {0};

USART_Handler_t handlerUSART2    = {0};

PWM_Handler_t handlerPWMControlServo = {0};
PWM_Handler_t handlerPWMControlMotor = {0};

I2C_Handler_t handlerI2C1 = {0};

EXTI_Config_t handlerExtiConfig0 = {0};


uint8_t rxData  = '\0';
uint8_t auxData = '\0';
char bufferData[64];
uint8_t flag = RESET;
uint8_t counterMotor = 0;
uint8_t duttyUp = 0;
uint8_t duttyDown = 0;
uint16_t freqUp = 0;
uint16_t freqDown = 0;
char bufferReception[64] = {0};
uint8_t counterReception = 0;
uint8_t doneTransaction = RESET;
uint8_t flagEx = RESET;
int16_t x = 0;
uint8_t adcFlag = RESET;
int16_t accelX      = 0;
int16_t accelY      = 0;

char cmd[32];
char userMsg[64];
unsigned int firstParameter;
unsigned int secondParameter;
unsigned int thirdParameter;
unsigned int adcData;


void initSystem(void);
void parseCommands(char *command);
void impulse(void);

int main (void){

		initSystem();

		while(1){

//			if (auxData != '\0'){
//				if (auxData == 'm'){
//					writeChar(&handlerUSART2, auxData);
//					if (handlerPWMControlMotor.ptrTIMx->CR1 & TIM_CR1_CEN){
//						stopPwmSignal(&handlerPWMControlMotor);
//						auxData = '\0';
//					}else {
//						startPwmSignal(&handlerPWMControlMotor);
//						auxData = '\0';
//					}
//
//				}else if (auxData == 's'){
//					writeChar(&handlerUSART2, auxData);
//					if (handlerPWMControlServo.ptrTIMx->CR1 & TIM_CR1_CEN){
//						stopPwmSignal(&handlerPWMControlServo);
//						auxData = '\0';
//					}else {
//						startPwmSignal(&handlerPWMControlServo);
//						auxData = '\0';
//					}
//				}
//
//
//
//				else if (auxData == 'd'){
//					duttyUp = handlerPWMControlMotor.config.duttyCicle;
//					duttyUp++;
//					sprintf(bufferData, "%u \n\r", duttyUp);
//					writeMsg(&handlerUSART2, bufferData);
//					if (duttyUp >= 100){
//						updateDuttyCycle(&handlerPWMControlMotor, 100);
//						auxData = '\0';
//					}else{
//						updateDuttyCycle(&handlerPWMControlMotor, duttyUp);
//						auxData = '\0';
//					}
//
//				}else if (auxData == 'u'){
//					duttyDown = handlerPWMControlMotor.config.duttyCicle;
//					duttyDown--;
//					sprintf(bufferData, "%u \n\r", duttyUp);
//					writeMsg(&handlerUSART2, bufferData);
//					if (duttyDown == 0){
//						updateDuttyCycle(&handlerPWMControlMotor, 1);
//						auxData = '\0';
//					}else{
//						updateDuttyCycle(&handlerPWMControlMotor, duttyDown);
//						auxData = '\0';
//					}
//
//				}
//
//				else if (auxData == 'a'){
//					writeChar(&handlerUSART2, auxData);
//					impulse();
//				}
//
//				else if (auxData == '-'){
//					duttyUp = handlerPWMControlServo.config.duttyCicle;
//					duttyUp++;
//					if (duttyUp >= 100){
//						updateDuttyCycle(&handlerPWMControlServo, 100);
//						auxData = '\0';
//					}else{
//						updateDuttyCycle(&handlerPWMControlServo, duttyUp);
//						auxData = '\0';
//					}
//
//				}else if (auxData == '+'){
//					duttyDown = handlerPWMControlServo.config.duttyCicle;
//					duttyDown--;
//					if (duttyDown == 0){
//						updateDuttyCycle(&handlerPWMControlServo, 1);
//						auxData = '\0';
//					}else{
//						updateDuttyCycle(&handlerPWMControlServo, duttyDown);
//						auxData = '\0';
//					}
//
//				}
//
//				else if (auxData == '1'){
//					writeChar(&handlerUSART2, auxData);
//					GPIO_WritePin(&handlerPinRele1, RESET);
//					GPIO_WritePin(&handlerPinRele2, RESET);
//					auxData = '\0';
//				}else if (auxData == '2'){
//					writeChar(&handlerUSART2, auxData);
//					GPIO_WritePin(&handlerPinRele1, SET);
//					GPIO_WritePin(&handlerPinRele2, SET);
//					auxData = '\0';
//
//				}
//			}
		//
		//
		//			else if (rxData == '-'){
		//				freqDown = (handlerPWMControlMotor.config.periodo);
		//				freqDown++;
		//
		//			if (freqDown == 0){
		//				updateFrequency(&handlerPWMControlMotor, 1);
		//			}else{
		//				updateFrequency(&handlerPWMControlMotor, freqDown);
		//				rxData = '\0';
		//			}
		//
		//			}else if (rxData == '+'){
		//					freqUp = (handlerPWMControlMotor.config.periodo);
		//					writeChar(&handlerUSART2, rxData);
		//					freqUp--;
		//
		//				if (freqUp == 0){
		//					updateFrequency(&handlerPWMControlMotor, 1);
		//				}else{
		//					updateFrequency(&handlerPWMControlMotor, freqUp);
		//					rxData = '\0';
		//				}
		//
		//			}
		//
		//
		//		}
		//		accelX = X_Accel(&handlerI2C1);
		//		accelY = Y_Accel(&handlerI2C1);
		//		sprintf(bufferData, "ACCELx = %u ACCELy = %u \n\r", accelX, accelY);
		//		writeMsg(&handlerUSART2, bufferData);


			x = adcData - 125;

			if (flagEx){
				impulse();
				flagEx = RESET;
			}

			if (adcFlag){

				if ((x < 30) & (x > -20)){
					if (handlerPWMControlMotor.config.duttyCicle == 95){
						__NOP();
					}else{
						updateDuttyCycle(&handlerPWMControlMotor, 95);
					}

				}else if (x >= 30){
					if (handlerPWMControlMotor.config.duttyCicle == 90){
						__NOP();
					}else{
					updateDuttyCycle(&handlerPWMControlMotor, 90);
					GPIO_WritePin(&handlerPinRele1, SET);
					GPIO_WritePin(&handlerPinRele2, SET);
					startPwmSignal(&handlerPWMControlMotor);
					}
				}else if (x <= -20){
					if (handlerPWMControlMotor.config.duttyCicle == 90){
						__NOP();
					}else{
					updateDuttyCycle(&handlerPWMControlMotor, 90);
					GPIO_WritePin(&handlerPinRele1, RESET);
					GPIO_WritePin(&handlerPinRele2, RESET);
					startPwmSignal(&handlerPWMControlMotor);
					}
				}

				adcFlag = RESET;
			}

			if (rxData != '\0'){
				bufferReception[counterReception] = rxData;
				counterReception++;

				if (rxData == '@'){
					doneTransaction = SET;
					writeMsg(&handlerUSART2, bufferReception);

					bufferReception[counterReception] = '\0';

					counterReception = 0;

				}

				rxData = '\0';

			}

			if (doneTransaction){
				parseCommands(bufferReception);
				doneTransaction = RESET;
			}


		}


}

void initSystem(void){
	//Comunicacion Serial por USART 2

	handlerUSARTPinTx.pGPIOx = GPIOA;
	handlerUSARTPinTx.GPIO_PinConfig.GPIO_PinAltFunMode = AF7;
	handlerUSARTPinTx.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	handlerUSARTPinTx.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerUSARTPinTx.GPIO_PinConfig.GPIO_PinNumber = PIN_2;
	handlerUSARTPinTx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerUSARTPinTx.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerUSARTPinTx);

	handlerUSARTPinRx.pGPIOx = GPIOA;
	handlerUSARTPinRx.GPIO_PinConfig.GPIO_PinAltFunMode = AF7;
	handlerUSARTPinRx.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	handlerUSARTPinRx.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerUSARTPinRx.GPIO_PinConfig.GPIO_PinNumber = PIN_3;
	handlerUSARTPinRx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerUSARTPinRx.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerUSARTPinRx);

	handlerUSART2.ptrUSARTx                      = USART2;
	handlerUSART2.USART_Config.USART_baudrate    = USART_BAUDRATE_9600;
	handlerUSART2.USART_Config.USART_enableInRx  = USART_INTERRUPT_RX_ENABLE;
	handlerUSART2.USART_Config.USART_enableInTx  = USART_INTERRUPT_TX_DISABLE;
	handlerUSART2.USART_Config.USART_mode        = USART_MODE_RXTX;
	handlerUSART2.USART_Config.USART_parity      = USART_PARITY_NONE;
	handlerUSART2.USART_Config.USART_stopbits    = USART_STOPBIT_1;
	handlerUSART2.USART_Config.USART_datasize    = USART_DATASIZE_8BIT;
	handlerUSART2.USART_Config.USART_parityError = 0;
	USART_Config(&handlerUSART2);


	//Blinky

	handlerLEDPin.pGPIOx = GPIOA;
	handlerLEDPin.GPIO_PinConfig.GPIO_PinAltFunMode = AF0;
	handlerLEDPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	handlerLEDPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerLEDPin.GPIO_PinConfig.GPIO_PinNumber = PIN_5;
	handlerLEDPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerLEDPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerLEDPin);
	GPIO_WritePin(&handlerLEDPin, SET);

	handlerBlinky.ptrTIMx = TIM2;
	handlerBlinky.TIMx_Config.TIMx_interruptEnable = 1;
	handlerBlinky.TIMx_Config.TIMx_mode            = BTIMER_MODE_UP;
	handlerBlinky.TIMx_Config.TIMx_period          = 2500;
	handlerBlinky.TIMx_Config.TIMx_speed           = BTIMER_SPEED_100us;
	BasicTimer_Config(&handlerBlinky);
	startTimer(&handlerBlinky);

	//PWM con sus respectivos pines del servomotor y del motor
	handlerPWMPinServo.pGPIOx = GPIOB;
	handlerPWMPinServo.GPIO_PinConfig.GPIO_PinAltFunMode = AF2;
	handlerPWMPinServo.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	handlerPWMPinServo.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerPWMPinServo.GPIO_PinConfig.GPIO_PinNumber = PIN_6;
	handlerPWMPinServo.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPWMPinServo.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPWMPinServo);

	handlerPWMPinMotor.pGPIOx = GPIOB;
	handlerPWMPinMotor.GPIO_PinConfig.GPIO_PinAltFunMode = AF2;
	handlerPWMPinMotor.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	handlerPWMPinMotor.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerPWMPinMotor.GPIO_PinConfig.GPIO_PinNumber = PIN_7;
	handlerPWMPinMotor.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPWMPinMotor.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPWMPinMotor);

	handlerPWMControlServo.ptrTIMx           = TIM4;
	handlerPWMControlServo.config.channel    = PWM_CHANNEL_1;
	handlerPWMControlServo.config.duttyCicle = 98;
	handlerPWMControlServo.config.periodo    = 20000;
	handlerPWMControlServo.config.prescaler  = PWM_SPEED_1us;
	pwm_Config(&handlerPWMControlServo);


	handlerPWMControlMotor.ptrTIMx           = TIM4;
	handlerPWMControlMotor.config.channel    = PWM_CHANNEL_2;
	handlerPWMControlMotor.config.duttyCicle = 95;
	handlerPWMControlMotor.config.periodo    = 20000;
	handlerPWMControlMotor.config.prescaler  = PWM_SPEED_1us;
	pwm_Config(&handlerPWMControlMotor);

	//I2C config

	handlerI2cSCL.pGPIOx                             = GPIOB;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinAltFunMode  = AF4;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinNumber      = PIN_8;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_OPENDRAIN;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PULLUP;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerI2cSCL);

	handlerI2cSDA.pGPIOx                             = GPIOB;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinAltFunMode  = AF4;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinNumber      = PIN_9;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_OPENDRAIN;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PULLUP;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerI2cSDA);

	handlerI2C1.ptrI2Cx = I2C1;
	handlerI2C1.modeI2C = I2C_MODE_FM;
	handlerI2C1.slaveAddress = ADDRESS_DOWN;
	i2c_config(&handlerI2C1);


	handlerExInSWPin.pGPIOx 							= GPIOB;
	handlerExInSWPin.GPIO_PinConfig.GPIO_PinNumber      = PIN_10;
	handlerExInSWPin.GPIO_PinConfig.GPIO_PinAltFunMode  = AF0;
	handlerExInSWPin.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_IN;
	handlerExInSWPin.GPIO_PinConfig.GPIO_PinPuPdControl =  GPIO_PUPDR_NOTHING;
	handlerExInSWPin.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerExInSWPin.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	handlerExtiConfig0.pGPIOHandler 		            = &handlerExInSWPin;
	handlerExtiConfig0.edgeType 			            = EXTERNAL_INTERRUPT_RISING_EDGE;
	extInt_Config(&handlerExtiConfig0);

	handlerPinRele1.pGPIOx = GPIOB;
	handlerPinRele1.GPIO_PinConfig.GPIO_PinAltFunMode = AF0;
	handlerPinRele1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	handlerPinRele1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerPinRele1.GPIO_PinConfig.GPIO_PinNumber = PIN_1;
	handlerPinRele1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinRele1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinRele1);
	GPIO_WritePin(&handlerPinRele1, RESET);

	handlerPinRele2.pGPIOx = GPIOB;
	handlerPinRele2.GPIO_PinConfig.GPIO_PinAltFunMode = AF0;
	handlerPinRele2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	handlerPinRele2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerPinRele2.GPIO_PinConfig.GPIO_PinNumber = PIN_2;
	handlerPinRele2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinRele2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinRele2);
	GPIO_WritePin(&handlerPinRele2, RESET);


	handlerTimADCConver.ptrTIMx = TIM3;
	handlerTimADCConver.TIMx_Config.TIMx_interruptEnable = 1;
	handlerTimADCConver.TIMx_Config.TIMx_mode = BTIMER_MODE_UP;
	handlerTimADCConver.TIMx_Config.TIMx_period = 100;
	handlerTimADCConver.TIMx_Config.TIMx_speed = BTIMER_SPEED_100us;
	BasicTimer_Config(&handlerTimADCConver);

	//ADC Config

	handlerADCJoy.channel = 0;
	handlerADCJoy.dataAlignment = ADC_ALIGNMENT_RIGHT;
	handlerADCJoy.resolution = ADC_RESOLUTION_8_BIT;
	handlerADCJoy.samplingPeriod = ADC_SAMPLING_PERIOD_28_CYCLES;
	adc_Config(&handlerADCJoy);
	startTimer(&handlerTimADCConver);

	config_SysTicksMs();
//	startAccel(&handlerI2C1);

}


void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerLEDPin);
}


void usart2Rx_Callback(void){
	rxData = getRxData();
	auxData = getRxData();
}

void callback_extInt10(void){
	flagEx = SET;
}

void adcComplete_Callback(void){
	adcData = getADC();
	adcFlag = SET;
}

void BasicTimer3_Callback(void){
	startSingleADC();
}

void parseCommands(char *command){

	sscanf(command, "%s %u %u %u %s", cmd ,&firstParameter, &secondParameter, &thirdParameter, userMsg);



	if (strcmp(cmd, "help") == 0){
		writeMsg(&handlerUSART2, "HELP MENU CMD :  \n");
		writeMsg(&handlerUSART2, "1)  startMotor   \n");
		writeMsg(&handlerUSART2, "2)  reverseMotor \n");
		writeMsg(&handlerUSART2, "3)  stopMotor    \n");
 		writeMsg(&handlerUSART2, "4)   \n");
		writeMsg(&handlerUSART2, "5)   \n");
		writeMsg(&handlerUSART2, "6)   \n");
		writeMsg(&handlerUSART2, "7)   \n");
		writeMsg(&handlerUSART2, "8)   \n");
		writeMsg(&handlerUSART2, "9)   \n");
		writeMsg(&handlerUSART2, "10)  \n");
		writeMsg(&handlerUSART2, "11)  \n");

	}else if (strcmp(cmd, "startMotor") == 0){
		updateDuttyCycle(&handlerPWMControlMotor, 48);
		startPwmSignal(&handlerPWMControlMotor);
	}
	else if (strcmp(cmd, "reverseMotor") == 0){
		stopPwmSignal(&handlerPWMControlMotor);
		GPIOxTooglePin(&handlerPinRele1);
		GPIOxTooglePin(&handlerPinRele2);
		delay_Ms(1000);
		startPwmSignal(&handlerPWMControlMotor);

	}
	else if (strcmp(cmd, "stopMotor") == 0){
		stopPwmSignal(&handlerPWMControlMotor);
	}
//	else if (com4 == 0){
//			updateFrequency(&handlerPwmR, 10);
//			updateFrequency(&handlerPwmG, 10);
//			updateFrequency(&handlerPwmB, 10);
//
//	}
//	else if (com5 == 0){
//		startOLED(&handlerI2C1);
//		drawMSG(&handlerI2C1, "BIENVENIDO!", 11);
//
//	}
//	else if (com6 == 0){
//		stopOLED(&handlerI2C1);
//
//	}
//	else if (com7 == 0){
//		uint8_t i =0;
//		uint8_t flag = RESET;
//		while (*(userMsg+i)!='\0'){
//			if ((*(userMsg+i) <= 122) & (*(userMsg+i) >= 97)){
//				flag = SET;
//				break;
//			}else{
//				__NOP();
//			}
//		}
//
//		if (flag){
//			writeMsg(&handlerUSART1, "REWRITE MESSAGE IN CAPITAL LETTER \n");
//		}else{
//			drawMSG(&handlerI2C1, userMsg, sizeof(userMsg));
//			flag = RESET;
//		}
//
//
//
//	}
//	else if (com10 == 0){
//		setScrollOUT = RESET;
//		if (firstParameter == SET){
//			setScrollIN =SET;
//
//		}else{
//			setScrollIN =RESET;
//		}
//
//
//	}
//	else if (com11 == 0){
//		clearDisplay(&handlerI2C1);
//		setScrollOUT = SET;
//
//
//	}
	else{
		writeMsg(&handlerUSART2, "WRONG CMD, WRITE IT AGAING \n");
	}

}

void impulse (void){
	updateDuttyCycle(&handlerPWMControlMotor, 90);
	delay_Ms(850);
	updateDuttyCycle(&handlerPWMControlMotor, 95);
//	updateDuttyCycle(&handlerPWMControlServo, 97);
//	delay_Ms(200);
//	updateDuttyCycle(&handlerPWMControlServo, 98);
}





