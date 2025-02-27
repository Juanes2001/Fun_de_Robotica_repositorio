/**
 ******************************************************************************
 * Examen_Tema_4
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

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// configuracion USART que se usara = Serial_1, 28800, EVEN

//handler para ADC
ADC_Config_t handlerADCJoy = {0};

//Pines handler para comunicación Serial
GPIO_Handler_t handlerPinTx = {0};
GPIO_Handler_t handlerPinRx = {0};

GPIO_Handler_t handlerPinR = {0};
GPIO_Handler_t handlerPinG = {0};
GPIO_Handler_t handlerPinB = {0};

GPIO_Handler_t handlerI2cSCL = {0};
GPIO_Handler_t handlerI2cSDA = {0};

//Pin del blinky
GPIO_Handler_t handlerLEDPin = {0};

//handler para USART1
USART_Handler_t handlerUSART1 = {0};

//handler para PWM

PWM_Handler_t handlerPwmR   = {0};
PWM_Handler_t handlerPwmG   = {0};
PWM_Handler_t handlerPwmB   = {0};

//Handlers de Timers
BasicTimer_Handler_t handlerLEDTim = {0};
BasicTimer_Handler_t handlerADCTim = {0};
BasicTimer_Handler_t handlerDateTim = {0};

//handlers I2C
I2C_Handler_t handlerI2C1 = {0};

//handler para el tiempo actual
RTC_Handler_t handlerRTC = {0};


void inSystem (void);
void parseCommands(char *stringVector);


//Variables necesarias para el programa
uint32_t adcData[2] ;
uint8_t adcFlag = RESET;
uint8_t rxData = '\0';
uint8_t counter = 0;
char cmd[32];
char userMsg[64];



unsigned int firstParameter;
unsigned int secondParameter;
unsigned int thirdParameter;

char bufferReception[64];

uint8_t counterReception = 0;
uint8_t doneTransaction = RESET;
uint8_t counterADC = 0;

uint8_t setPWM = RESET;

uint8_t duttyUpR = 0;
uint8_t duttyUpG = 0;
uint8_t duttyUpB1 = 0;
uint8_t duttyUpB2 = 0;
uint8_t duttyDownR = 0;
uint8_t duttyDownG1 = 0;
uint8_t duttyDownG2 = 0;
uint8_t duttyDownB = 0;
uint16_t vectorArcUp = 0;
uint16_t vectorArcDown = 0;

int16_t x = 0;
int16_t y = 0;

uint8_t com1 = 0;
uint8_t com2 = 0;
uint8_t com3 = 0;
uint8_t com4 = 0;
uint8_t com5 = 0;
uint8_t com6 = 0;
uint8_t com7 = 0;
uint8_t com8 = 0;
uint8_t com9 = 0;
uint8_t com10 = 0;
uint8_t com11 = 0;

uint8_t flagDate = RESET;

char date[16];

uint8_t  sec = 0;
uint8_t  min = 0;
uint8_t  hours = 0;

uint8_t setScrollIN = RESET;
uint8_t setScrollOUT = RESET;
uint8_t setScrollUP = RESET;
uint8_t setScrollDOWN = RESET;


//ADDRESS OLED

#define ADDRESS 0b0111100

//Este es el corazon del programa donde se ejecuta todo
int main(void){

	inSystem();


	while(1){



		x = adcData[0]-2000;
		y = adcData[1]-2000;
		vectorArcUp = (180*atan2(y,x))/M_PI;
		vectorArcDown =180+(180*atan2(-y,-x))/M_PI;


		duttyUpR = (100*(vectorArcDown + 120))/120;
		duttyUpG =  (100*vectorArcUp)/120;
		duttyUpB1 = (100*(vectorArcUp-120))/120;
		duttyUpB2 = (100*(vectorArcDown-120))/120;
		duttyDownR = 100-(100*vectorArcUp)/120;
		duttyDownG1 = 100-(100*(vectorArcUp-120))/120;
		duttyDownG2 = 100-(100*(vectorArcDown-120))/120;
		duttyDownB = 100-(100*(vectorArcDown+120))/120;


		if (setScrollUP){
			counter++;
			setLineAddress(&handlerI2C1, counter);
			if (counter == 64){
				counter = 0;
			}
			setScrollUP=RESET;
		}else if (setScrollDOWN){
			setLineAddress(&handlerI2C1, 64-counter);
			counter++;
			if (counter == 63){
				counter = 0;
			}
			setScrollUP=RESET;

		}


		if (adcFlag){


			if (((x>-50) & (x<50)) & ((y>-50) & (y<50)) ){
				updateDuttyCycle(&handlerPwmR, 100);
				updateDuttyCycle(&handlerPwmG, 100);
				updateDuttyCycle(&handlerPwmB, 100);


			}else{
				if ((vectorArcUp > 0) & (vectorArcUp < 120) ){
					updateDuttyCycle(&handlerPwmR, duttyDownR);
					updateDuttyCycle(&handlerPwmG, duttyUpG);
					updateDuttyCycle(&handlerPwmB, 0);
				}else if ((vectorArcUp > 120) & (vectorArcUp < 180) ){
					updateDuttyCycle(&handlerPwmR, 0);
					updateDuttyCycle(&handlerPwmG, duttyDownG1);
					updateDuttyCycle(&handlerPwmB, duttyUpB1);
				}else if ((vectorArcDown > 180) & (vectorArcDown < 240) ){
					updateDuttyCycle(&handlerPwmR, 0);
					updateDuttyCycle(&handlerPwmG, duttyDownG2);
					updateDuttyCycle(&handlerPwmB, duttyUpB2);
				}else if ((vectorArcDown > 240) & (vectorArcDown < 380) ){
					updateDuttyCycle(&handlerPwmR, duttyUpR);
					updateDuttyCycle(&handlerPwmG, 0);
					updateDuttyCycle(&handlerPwmB, duttyDownB);
				}
			}
		}



		//COMANDOS


		if (rxData != '\0'){
			bufferReception[counterReception] = rxData;
			counterReception++;

			if (rxData == '@'){
				doneTransaction = SET;

				bufferReception[counterReception] = '\0';

				counterReception = 0;

			}

			rxData = '\0';

		}

		if (doneTransaction){
			parseCommands(bufferReception);
			doneTransaction = RESET;
		}

		if (flagDate){
			sec = *(getDate());
			min = *(getDate()+1);
			hours = *(getDate()+2);

			sprintf(date, "%u:%u:%u",hours,min,sec);
			drawMSG(&handlerI2C1, date, sizeof(date));
			flagDate = RESET;
		}




	}
}

void inSystem (void){

	//Descripcion de la configuracion

	//BLINKY LED

	handlerLEDPin.pGPIOx = GPIOA;
	handlerLEDPin.GPIO_PinConfig.GPIO_PinAltFunMode = AF0;
	handlerLEDPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	handlerLEDPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSHPULL;
	handlerLEDPin.GPIO_PinConfig.GPIO_PinNumber = PIN_5;
	handlerLEDPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerLEDPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerLEDPin);
	GPIO_WritePin(&handlerLEDPin, SET);

	handlerLEDTim.ptrTIMx = TIM2;
	handlerLEDTim.TIMx_Config.TIMx_interruptEnable = 1;
	handlerLEDTim.TIMx_Config.TIMx_mode = BTIMER_MODE_UP;
	handlerLEDTim.TIMx_Config.TIMx_period = 2500;
	handlerLEDTim.TIMx_Config.TIMx_speed = BTIMER_SPEED_100us;
	BasicTimer_Config(&handlerLEDTim);

	startTimer(&handlerLEDTim);


	//Conversion del JOYSTICK
	handlerADCTim.ptrTIMx = TIM4;
	handlerADCTim.TIMx_Config.TIMx_interruptEnable = 1;
	handlerADCTim.TIMx_Config.TIMx_mode = BTIMER_MODE_UP;
	handlerADCTim.TIMx_Config.TIMx_period = 100;
	handlerADCTim.TIMx_Config.TIMx_speed = BTIMER_SPEED_100us;
	BasicTimer_Config(&handlerADCTim);


    handlerADCJoy.channelVector[0] = 0;
    handlerADCJoy.channelVector[1] = 1;
    handlerADCJoy.dataAlignment = ADC_ALIGNMENT_RIGHT;
    handlerADCJoy.resolution = ADC_RESOLUTION_12_BIT;
    handlerADCJoy.samplingPeriod = ADC_SAMPLING_PERIOD_28_CYCLES;
    ADC_ConfigMultichannel(&handlerADCJoy, 2);

    //Comunicacion Serial
    handlerPinTx.pGPIOx                             = GPIOA;
    handlerPinTx.GPIO_PinConfig.GPIO_PinAltFunMode  = AF7;
    handlerPinTx.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
    handlerPinTx.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
    handlerPinTx.GPIO_PinConfig.GPIO_PinNumber      = PIN_9;
    handlerPinTx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
    handlerPinTx.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinTx);


	handlerPinRx.pGPIOx                             = GPIOA;
	handlerPinRx.GPIO_PinConfig.GPIO_PinAltFunMode  = AF7;
	handlerPinRx.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerPinRx.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerPinRx.GPIO_PinConfig.GPIO_PinNumber      = PIN_10;
	handlerPinRx.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinRx.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinRx);

	handlerUSART1.ptrUSARTx                      = USART1;
	handlerUSART1.USART_Config.USART_baudrate    = USART_BAUDRATE_9600;//37,7222 de Mantiza
	handlerUSART1.USART_Config.USART_enableInRx  = USART_INTERRUPT_RX_ENABLE;
	handlerUSART1.USART_Config.USART_enableInTx  = USART_INTERRUPT_TX_DISABLE;
	handlerUSART1.USART_Config.USART_mode        = USART_MODE_RXTX;
	handlerUSART1.USART_Config.USART_parity      = USART_PARITY_ODD;
	handlerUSART1.USART_Config.USART_stopbits    = USART_STOPBIT_1;
	handlerUSART1.USART_Config.USART_datasize    = USART_DATASIZE_9BIT;
	handlerUSART1.USART_Config.USART_parityError = 0;
	USART_Config(&handlerUSART1);


	//Configuracion de pines y timers para PWM


	handlerPinR.pGPIOx                             = GPIOC;
	handlerPinR.GPIO_PinConfig.GPIO_PinAltFunMode  = AF2;
	handlerPinR.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerPinR.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerPinR.GPIO_PinConfig.GPIO_PinNumber      = PIN_7;
	handlerPinR.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinR.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinR);


	handlerPinG.pGPIOx                             = GPIOC;
	handlerPinG.GPIO_PinConfig.GPIO_PinAltFunMode  = AF2;
	handlerPinG.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerPinG.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerPinG.GPIO_PinConfig.GPIO_PinNumber      = PIN_8;
	handlerPinG.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinG.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinG);


	handlerPinB.pGPIOx                             = GPIOC;
	handlerPinB.GPIO_PinConfig.GPIO_PinAltFunMode  = AF2;
	handlerPinB.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerPinB.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerPinB.GPIO_PinConfig.GPIO_PinNumber      = PIN_9;
	handlerPinB.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerPinB.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinB);



	handlerPwmR.ptrTIMx           = TIM3;
	handlerPwmR.config.channel    = PWM_CHANNEL_2;
	handlerPwmR.config.duttyCicle = 100;
	handlerPwmR.config.periodo    = 100;
	handlerPwmR.config.prescaler  = BTIMER_SPEED_100us;
	pwm_Config(&handlerPwmR);


	handlerPwmG.ptrTIMx           = TIM3;
	handlerPwmG.config.channel    = PWM_CHANNEL_3;
	handlerPwmG.config.duttyCicle = 100;
	handlerPwmG.config.periodo    = 100;
	handlerPwmG.config.prescaler  = BTIMER_SPEED_100us;
	pwm_Config(&handlerPwmG);

	handlerPwmB.ptrTIMx           = TIM3;
	handlerPwmB.config.channel    = PWM_CHANNEL_4;
	handlerPwmB.config.duttyCicle = 100;
	handlerPwmB.config.periodo    = 100;
	handlerPwmB.config.prescaler  = BTIMER_SPEED_100us;
	pwm_Config(&handlerPwmB);


    //Configuracion del RTC
	handlerRTC.RTC_config.rtcDay = 8;
	handlerRTC.RTC_config.rtcHours = 0;
	handlerRTC.RTC_config.rtcMinutes = 0;
	handlerRTC.RTC_config.rtcMonth = 11;
	handlerRTC.RTC_config.rtcSeconds =0;
	handlerRTC.RTC_config.rtcWeekDay = THUESDAY;
	handlerRTC.RTC_config.rtcYear= 22;
	Rtc_Congif(&handlerRTC);

	//OLED display por comunicacion I2C

	handlerI2cSCL.pGPIOx                             = GPIOB;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinAltFunMode  = AF4;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinNumber      = PIN_6;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_OPENDRAIN;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PULLUP;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerI2cSCL);

	handlerI2cSDA.pGPIOx                             = GPIOB;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinAltFunMode  = AF4;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinNumber      = PIN_7;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_OPENDRAIN;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_PULLUP;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerI2cSDA);

	handlerI2C1.ptrI2Cx = I2C1;
	handlerI2C1.modeI2C = I2C_MODE_FM;
	handlerI2C1.slaveAddress = ADDRESS;
	i2c_config(&handlerI2C1);

	//Timer 5 para interrupciones del comando para cronometro



	handlerDateTim.ptrTIMx = TIM5;
	handlerDateTim.TIMx_Config.TIMx_interruptEnable = 1;
	handlerDateTim.TIMx_Config.TIMx_mode = BTIMER_MODE_UP;
	handlerDateTim.TIMx_Config.TIMx_period = 10000;
	handlerDateTim.TIMx_Config.TIMx_speed = BTIMER_SPEED_100us;
	BasicTimer_Config(&handlerDateTim);

	startTimer(&handlerDateTim);




}


// Callback para el blynky y el comando Scroll UP y DOWN
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerLEDPin);
	if (!(setScrollOUT)){

		if (setScrollIN){
			setScrollUP = SET;
		}else{
			setScrollDOWN = SET;
		}
	}else{
		__NOP();
	}
}

//Callback para el comando del cronometro

void BasicTimer5_Callback(void){
	flagDate = SET;
}

//Callback para interrupciones posterior a la multiconversion

void adcComplete_Callback(void){
	counterADC++;
	if ((counterADC % 2) != 0){
		adcData[0] = getADC();
	}else if ((counterADC % 2) == 0){
		adcData[1] = getADC();
	}

	if ((counterADC % 2) == 0){
		adcFlag = SET;
	}


}

//Calback para recepciones por coolterm

void usart1Rx_Callback(void){
	rxData = getRxData();
}

//Callback para comando de setear conversion ADC y LED RGB

void BasicTimer4_Callback(void){
	startSingleADC();
}


//Funcion ejecutadora de comandos

void parseCommands(char *stringVector){

	sscanf(stringVector, "%s %u %u %u %s", cmd ,&firstParameter, &secondParameter, &thirdParameter, userMsg);
	com1 = strcmp(cmd, "startLED") ;
	com2 = strcmp(cmd, "stopLED");
	com3 = strcmp(cmd, "setLED_period");
	com4 = strcmp(cmd, "resetLED_period");
	com5 = strcmp(cmd, "start_display");
	com6 = strcmp(cmd, "stop_display");
	com7 = strcmp(cmd, "print_msg");
	com8 = strcmp(cmd, "print_date");
	com9 = strcmp(cmd, "set_cronometer");
	com10 = strcmp(cmd, "scroll");
	com11 = strcmp(cmd, "clear_dysplay");



	if (strcmp(cmd, "help") == 0){
		writeMsg(&handlerUSART1, "HELP MENU CMD : \n");
		writeMsg(&handlerUSART1, "1)  startLED \n");
		writeMsg(&handlerUSART1, "2)  stopLED \n");
		writeMsg(&handlerUSART1, "3)  setLED_period #RnewPeriod #GnewPeriod #BnewPeriod \n");
 		writeMsg(&handlerUSART1, "4)  resetLED_period \n");
		writeMsg(&handlerUSART1, "5)  start_display \n");
		writeMsg(&handlerUSART1, "6)  stop_display \n");
		writeMsg(&handlerUSART1, "7)  print_msg WRITE_MSG_WITH_CAPITAL_LETTERS \n");                                          //7
		writeMsg(&handlerUSART1, "8)  print_date \n");
		writeMsg(&handlerUSART1, "9)  set_cronometer #hours #minutes #seconds \n");
		writeMsg(&handlerUSART1, "10) scroll #1=UP or 0=DOWN \n");
		writeMsg(&handlerUSART1, "11) clear_dysplay \n");

	}
	else if (com1 == 0){
		startTimer(&handlerADCTim);
		startPwmSignal(&handlerPwmR);
		startPwmSignal(&handlerPwmG);
		startPwmSignal(&handlerPwmB);
		setPWM = SET;


	}
	else if (com2  == 0){
		stopTimer(&handlerADCTim);
		stopPwmSignal(&handlerPwmR);
		stopPwmSignal(&handlerPwmG);
		stopPwmSignal(&handlerPwmB);
		setPWM = RESET;
	}
	else if (com3 == 0){
		if (setPWM){
			updateFrequency(&handlerPwmR, firstParameter);
			updateFrequency(&handlerPwmG, secondParameter);
			updateFrequency(&handlerPwmB, thirdParameter);
		}else{
			writeMsg(&handlerUSART1, "Turn on the LED first \n");
		}
	}
	else if (com4 == 0){
			updateFrequency(&handlerPwmR, 10);
			updateFrequency(&handlerPwmG, 10);
			updateFrequency(&handlerPwmB, 10);

	}
	else if (com5 == 0){
		startOLED(&handlerI2C1);
		drawMSG(&handlerI2C1, "BIENVENIDO!", 11);

	}
	else if (com6 == 0){
		stopOLED(&handlerI2C1);

	}
	else if (com7 == 0){
		uint8_t i =0;
		uint8_t flag = RESET;
		while (*(userMsg+i)!='\0'){
			if ((*(userMsg+i) <= 122) & (*(userMsg+i) >= 97)){
				flag = SET;
				break;
			}else{
				__NOP();
			}
		}

		if (flag){
			writeMsg(&handlerUSART1, "REWRITE MESSAGE IN CAPITAL LETTER \n");
		}else{
			drawMSG(&handlerI2C1, userMsg, sizeof(userMsg));
			flag = RESET;
		}



	}
	else if (com8 == 0){
		handlerRTC.RTC_config.rtcDay = 8;
		handlerRTC.RTC_config.rtcHours = firstParameter;
		handlerRTC.RTC_config.rtcMinutes = secondParameter;
		handlerRTC.RTC_config.rtcMonth = 11;
		handlerRTC.RTC_config.rtcSeconds =thirdParameter;
		handlerRTC.RTC_config.rtcWeekDay = THUESDAY;
		handlerRTC.RTC_config.rtcYear= 22;
		Rtc_Congif(&handlerRTC);

		sprintf(date,"%u/%u/%u%u:%u:%u",22,11,8,firstParameter,secondParameter,thirdParameter);

		drawMSG(&handlerI2C1, date, sizeof(date));


	}
	else if (com9 == 0){
		handlerRTC.RTC_config.rtcDay = 8;
		handlerRTC.RTC_config.rtcHours = 0;
		handlerRTC.RTC_config.rtcMinutes = 0;
		handlerRTC.RTC_config.rtcMonth = 11;
		handlerRTC.RTC_config.rtcSeconds =0;
		handlerRTC.RTC_config.rtcWeekDay = THUESDAY;
		handlerRTC.RTC_config.rtcYear= 22;
		Rtc_Congif(&handlerRTC);

		startTimer(&handlerDateTim);


	}
	else if (com10 == 0){
		setScrollOUT = RESET;
		if (firstParameter == SET){
			setScrollIN =SET;

		}else{
			setScrollIN =RESET;
		}


	}
	else if (com11 == 0){
		clearDisplay(&handlerI2C1);
		setScrollOUT = SET;


	}
	else{
		writeMsg(&handlerUSART1, "WRONG CMD, WRITE IT AGAING \n");
	}

}

//Revisión Parcial
//- La terminal funciona, pero no fue posible probar algunos comandos
//- El Joystick funciona
//- El RGB funciona y responde muy bien al movimiento del joystick
//- No presenta el RTC
//- Presenta la OLED, pero por algun motivo no muestra caracteres.
//-
//
//
//(8+20+0+10+10+20)*5/100
//
//Calificación = 3.4
