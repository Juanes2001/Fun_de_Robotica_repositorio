/**
 ******************************************************************************
 * Solucion_Tarea4
 * Autor: Juan Esteban Rodriguez Ochoa
 * Correo: jrodriguezoc@unal.edu.co
 ******************************************************************************
 ******************************************************************************
 */
#include "BasicTimer.h"
#include "GPIOxDriver.h"
#include "USARTxDriver.h"
#include "EXTIDriver.h"
#include <stdint.h>
#include <stdbool.h>
// Definición de variables de estado, contador, y strings para el desarrollo de la tarea
/*
 * clk es la variable de estado que almacena el estado (0 o 1) de la señal CLOCK del Encoder.
 * data es la variable de estado que almacena el estado (0 o 1) de la señal DATA del Encoder.
 * buffer1 almacena el mensaje de si se hizo el giro clock wise o counter clock wise en el Encoder y el contador.
 * buffer2 almacena el mensaje de la interrupcion propia del boton externo.
 * counter es el contador.
 * msgButton es la variable de estado de la señal generada por la configuracion PULLDOWN del boton externo.
 */
uint8_t clk = 0;
uint8_t data = 0;
char buffer1[64];
char buffer2[200];
uint8_t counter = 0;
uint8_t msgButton = 0;

//Creamos una función que se encargara de la configguiracion de todos los perifericos que usaremos ,
//los cuales incluyen pines GPIO, Timers, USART, y los EXTI. La explicación de cada uno se hara dentro
//de la definición completa de inSystem();
void inSystem (void);


GPIO_Handler_t handlerBlinkyLed        = {0}; //. PA5 --
BasicTimer_Handler_t handlerTIM2       = {0}; //.

GPIO_Handler_t handlerExInButtonPin    = {0}; //. PB1 --
EXTI_Config_t handlerExtiConfig1       = {0}; //.

GPIO_Handler_t handlerExInClock        = {0}; //. PA0 --
GPIO_Handler_t handlerExInData         = {0}; //. PA1 --
EXTI_Config_t handlerExtiConfig0       = {0}; //.

GPIO_Handler_t handlerPinTx            = {0}; //. PA2 --
USART_Handler_t handlerUSART2          = {0}; //.

GPIO_Handler_t handlerPinTran1         = {0}; //. PA6 --
GPIO_Handler_t handlerPinTran2         = {0}; //. PA7 --
BasicTimer_Handler_t handlerTIM5       = {0};


GPIO_Handler_t handlerPinSeg1         = {0}; //. PC1 --
GPIO_Handler_t handlerPinSeg2         = {0}; //. PC2 --
GPIO_Handler_t handlerPinSeg3         = {0}; //. PC3 --
GPIO_Handler_t handlerPinSeg4         = {0}; //. PC4 --
GPIO_Handler_t handlerPinSeg5         = {0}; //. PC5 --
GPIO_Handler_t handlerPinSeg6         = {0}; //. PC6 --
GPIO_Handler_t handlerPinSeg7         = {0}; //. PC7 --


int main (void){
	//Iniciamos todas las configuraciones.
	inSystem();

   // Loop infinito
	while(1){

		//PUNTO 3 y 5
		//Es CW cuando el CLOCK se encuentra en estado 1 (rising edge) y el DATA se encuentre en esatdo 0
		// pero cuando DATA se haya en estado 1 al igual que CLOCK, se trata de CCW. cuando se detecta que caso
		// de giro es, se envia el mensaje CW o CCW direction = el valor del contador.
		if (clk == 1 && data == 0){
			counter++;
			if (counter <= 50 && counter>=0){
				sprintf(buffer1, "CW direction = %u \n", counter);
				writeMsg(&handlerUSART2, buffer1);
			}else{
				counter = 0;
				sprintf(buffer1, "CW direction = %u \n", counter);
				writeMsg(&handlerUSART2, buffer1);
			}
			clk = 0;

		}else if (clk == 1 && data == 1){
			counter--;
			if (counter <= 50 && counter>=0){
				sprintf(buffer1, "CCW direction = %u \n", counter);
				writeMsg(&handlerUSART2, buffer1);
			}else{
				counter = 50;
				sprintf(buffer1, "CCW direction = %u \n", counter);
				writeMsg(&handlerUSART2, buffer1);
			}
			clk = 0;
			data = 0;
		}else{
			__NOP();
		}

		// PUNTO 6
		// El boton regguetonero, cada vez que es pulsado, detecta por el PIN B1 gracias al EXTI 1 el flanco de
		// subida de la señal. Por lo que por comunicacon serial, manda un mensaje de una de mis estrofas
		// favoritas de la cancion Efecto de BadBunny.
		if (msgButton == 1){
			char buffer2[200] = "PORQUE LA NOTA YA ESTA HACIENDO EFECTOOO \r MI MUNDO ESTA AL DIA Y ME "
					"SIENTO PERFECTO\r PORQUE ESTAS TU ASI MOVIENDOTE ASI NO PARE \r BBY TU ERE MI DROGA ESTA "
					"NOCHE NO LE BAJE \r";
			writeMsg(&handlerUSART2, buffer2);
			msgButton = 0;

		}



		// PUNTO 4
		// Se hace un switch-case para estudiar el caso de cada numero, y asi hacer los cambios respectivos en los
		// estados de los pines para apagar estrategicamente los segmentos y mostrar los numeros deseados
		switch (counter){
			case 0: {

				GPIO_WritePin(&handlerPinSeg1, RESET);
				GPIO_WritePin(&handlerPinSeg2, RESET);
				GPIO_WritePin(&handlerPinSeg3, RESET);
				GPIO_WritePin(&handlerPinSeg4, SET);
				GPIO_WritePin(&handlerPinSeg5, RESET);
				GPIO_WritePin(&handlerPinSeg6, RESET);
				GPIO_WritePin(&handlerPinSeg7, RESET);


				break;
			}
			case 1: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}
				break;
			}
			case 2: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, SET);

				}else{

					__NOP();

				}
				break;
			}
			case 3: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

					__NOP();

				}

				break;
			}
			case 4: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

						GPIO_WritePin(&handlerPinSeg1, RESET);
						GPIO_WritePin(&handlerPinSeg2, RESET);
						GPIO_WritePin(&handlerPinSeg3, RESET);
						GPIO_WritePin(&handlerPinSeg4, SET);
						GPIO_WritePin(&handlerPinSeg5, RESET);
						GPIO_WritePin(&handlerPinSeg6, RESET);
						GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


						GPIO_WritePin(&handlerPinSeg1, RESET);
						GPIO_WritePin(&handlerPinSeg2, SET);
						GPIO_WritePin(&handlerPinSeg3, RESET);
						GPIO_WritePin(&handlerPinSeg4, RESET);
						GPIO_WritePin(&handlerPinSeg5, SET);
						GPIO_WritePin(&handlerPinSeg6, SET);
						GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 5: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

						GPIO_WritePin(&handlerPinSeg1, RESET);
						GPIO_WritePin(&handlerPinSeg2, RESET);
						GPIO_WritePin(&handlerPinSeg3, RESET);
						GPIO_WritePin(&handlerPinSeg4, SET);
						GPIO_WritePin(&handlerPinSeg5, RESET);
						GPIO_WritePin(&handlerPinSeg6, RESET);
						GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


						GPIO_WritePin(&handlerPinSeg1, SET);
						GPIO_WritePin(&handlerPinSeg2, RESET);
						GPIO_WritePin(&handlerPinSeg3, RESET);
						GPIO_WritePin(&handlerPinSeg4, RESET);
						GPIO_WritePin(&handlerPinSeg5, RESET);
						GPIO_WritePin(&handlerPinSeg6, SET);
						GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 6: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

						GPIO_WritePin(&handlerPinSeg1, RESET);
						GPIO_WritePin(&handlerPinSeg2, RESET);
						GPIO_WritePin(&handlerPinSeg3, RESET);
						GPIO_WritePin(&handlerPinSeg4, SET);
						GPIO_WritePin(&handlerPinSeg5, RESET);
						GPIO_WritePin(&handlerPinSeg6, RESET);
						GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


						GPIO_WritePin(&handlerPinSeg1, SET);
						GPIO_WritePin(&handlerPinSeg2, RESET);
						GPIO_WritePin(&handlerPinSeg3, RESET);
						GPIO_WritePin(&handlerPinSeg4, RESET);
						GPIO_WritePin(&handlerPinSeg5, RESET);
						GPIO_WritePin(&handlerPinSeg6, RESET);
						GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 7: {

				if (!(GPIO_ReadPin(&handlerPinTran1))){

						GPIO_WritePin(&handlerPinSeg1, RESET);
						GPIO_WritePin(&handlerPinSeg2, RESET);
						GPIO_WritePin(&handlerPinSeg3, RESET);
						GPIO_WritePin(&handlerPinSeg4, SET);
						GPIO_WritePin(&handlerPinSeg5, RESET);
						GPIO_WritePin(&handlerPinSeg6, RESET);
						GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


						GPIO_WritePin(&handlerPinSeg1, RESET);
						GPIO_WritePin(&handlerPinSeg2, RESET);
						GPIO_WritePin(&handlerPinSeg3, SET);
						GPIO_WritePin(&handlerPinSeg4, SET);
						GPIO_WritePin(&handlerPinSeg5, SET);
						GPIO_WritePin(&handlerPinSeg6, SET);
						GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 8: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

						GPIO_WritePin(&handlerPinSeg1, RESET);
						GPIO_WritePin(&handlerPinSeg2, RESET);
						GPIO_WritePin(&handlerPinSeg3, RESET);
						GPIO_WritePin(&handlerPinSeg4, SET);
						GPIO_WritePin(&handlerPinSeg5, RESET);
						GPIO_WritePin(&handlerPinSeg6, RESET);
						GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


						GPIO_WritePin(&handlerPinSeg1, RESET);
						GPIO_WritePin(&handlerPinSeg2, RESET);
						GPIO_WritePin(&handlerPinSeg3, RESET);
						GPIO_WritePin(&handlerPinSeg4, RESET);
						GPIO_WritePin(&handlerPinSeg5, RESET);
						GPIO_WritePin(&handlerPinSeg6, RESET);
						GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}

			case 9: {

				if (!(GPIO_ReadPin(&handlerPinTran1))){

						GPIO_WritePin(&handlerPinSeg1, RESET);
						GPIO_WritePin(&handlerPinSeg2, RESET);
						GPIO_WritePin(&handlerPinSeg3, RESET);
						GPIO_WritePin(&handlerPinSeg4, SET);
						GPIO_WritePin(&handlerPinSeg5, RESET);
						GPIO_WritePin(&handlerPinSeg6, RESET);
						GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


						GPIO_WritePin(&handlerPinSeg1, RESET);
						GPIO_WritePin(&handlerPinSeg2, RESET);
						GPIO_WritePin(&handlerPinSeg3, RESET);
						GPIO_WritePin(&handlerPinSeg4, RESET);
						GPIO_WritePin(&handlerPinSeg5, RESET);
						GPIO_WritePin(&handlerPinSeg6, SET);
						GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}
				break;

			}
			case 10: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

						GPIO_WritePin(&handlerPinSeg1, RESET);
						GPIO_WritePin(&handlerPinSeg2, SET);
						GPIO_WritePin(&handlerPinSeg3, SET);
						GPIO_WritePin(&handlerPinSeg4, SET);
						GPIO_WritePin(&handlerPinSeg5, SET);
						GPIO_WritePin(&handlerPinSeg6, SET);
						GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


						GPIO_WritePin(&handlerPinSeg1, RESET);
						GPIO_WritePin(&handlerPinSeg2, RESET);
						GPIO_WritePin(&handlerPinSeg3, RESET);
						GPIO_WritePin(&handlerPinSeg4, SET);
						GPIO_WritePin(&handlerPinSeg5, RESET);
						GPIO_WritePin(&handlerPinSeg6, RESET);
						GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;

			}
			case 11: {

				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);
				}else{

						__NOP();

				}

				break;
			}
			case 12: {

				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, SET);
				}else{

						__NOP();

				}


				break;
			}
			case 13: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 14: {

				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 15: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, SET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 16: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, SET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 17: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);



				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 18: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 19: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}

			case 20: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, SET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;

			}
			case 21: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, SET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;

			}
			case 22: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, SET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, SET);

				}else{

						__NOP();

				}

				break;
			}
			case 23: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, SET);

				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 24: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, SET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 25: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, SET);

				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, SET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 26: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, SET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, SET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 27: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, SET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 28: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, SET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 29: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, SET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 30: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}

			case 31: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;

			}
			case 32: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, SET);

				}else{

						__NOP();

				}

				break;

			}
			case 33: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}


				break;
			}
			case 34: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 35: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, SET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 36: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, SET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 37: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 38: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 39: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 40: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 41: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}

			case 42: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, SET);

				}else{

						__NOP();

				}

				break;

			}
			case 43: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;

			}
			case 44: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 45: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, SET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 46: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, SET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 47: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, SET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}
			case 48: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}

			case 49: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, SET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, SET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;

			}
			case 50: {
				if (!(GPIO_ReadPin(&handlerPinTran1))){

					GPIO_WritePin(&handlerPinSeg1, SET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, RESET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, SET);
					GPIO_WritePin(&handlerPinSeg7, RESET);


				}else if (!(GPIO_ReadPin(&handlerPinTran2))){


					GPIO_WritePin(&handlerPinSeg1, RESET);
					GPIO_WritePin(&handlerPinSeg2, RESET);
					GPIO_WritePin(&handlerPinSeg3, RESET);
					GPIO_WritePin(&handlerPinSeg4, SET);
					GPIO_WritePin(&handlerPinSeg5, RESET);
					GPIO_WritePin(&handlerPinSeg6, RESET);
					GPIO_WritePin(&handlerPinSeg7, RESET);

				}else{

						__NOP();

				}

				break;
			}

			default :{
				__NOP();
				break;
			}

		}




	}
}


void inSystem (void){

	// A continuación se explicarán las configuraciones de todos los perifericos usados en la tarea.

	////////////////////////////-----LED de estado o Bliky LED----//////////////////////////////////////

	/*
	 * Se configuro el Timer 2 y el pin conectado al LED de estado del MCU (PA5) para hacer el blinky, usando las interrupciones de los timers
	 */

	handlerBlinkyLed.pGPIOx = GPIOA;
	handlerBlinkyLed.GPIO_PinConfig.GPIO_PinNumber      = PIN_5;
	handlerBlinkyLed.GPIO_PinConfig.GPIO_PinAltFunMode  = AF0;
	handlerBlinkyLed.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
	handlerBlinkyLed.GPIO_PinConfig.GPIO_PinPuPdControl =  GPIO_PUPDR_NOTHING;
	handlerBlinkyLed.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerBlinkyLed.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerBlinkyLed);

	handlerTIM2.ptrTIMx = TIM2;
	handlerTIM2.TIMx_Config.TIMx_mode               = BTIMER_MODE_UP;
	handlerTIM2.TIMx_Config.TIMx_speed              = BTIMER_SPEED_100us;
	handlerTIM2.TIMx_Config.TIMx_period             = 2500;
	handlerTIM2.TIMx_Config.TIMx_interruptEnable    = 1;
	BasicTimer_Config(&handlerTIM2);

	startTimer (&handlerTIM2);

	//////////////////////////////-------------------------------------///////////////////////////////////////

	//////////////////////////------Configuracion de Pines para el EXTI------/////////////////////////////////


	/*
	 * Se configuran los pines PA0 y PA1 para leer las señales del Encoder,
	 * el pin PA0 como una entrada simple para el EXTI 0 y usar la interrupcion del mismo
	 * y el PA1 como un pin simple para leer su estado.
	 */


	handlerExInClock.pGPIOx 							= GPIOA;
	handlerExInClock.GPIO_PinConfig.GPIO_PinNumber      = PIN_0;
	handlerExInClock.GPIO_PinConfig.GPIO_PinAltFunMode  = AF0;
	handlerExInClock.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_IN;
	handlerExInClock.GPIO_PinConfig.GPIO_PinPuPdControl =  GPIO_PUPDR_NOTHING;
	handlerExInClock.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerExInClock.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	handlerExtiConfig0.pGPIOHandler 		            = &handlerExInClock;
	handlerExtiConfig0.edgeType 			            = EXTERNAL_INTERRUPT_RISING_EDGE;
	extInt_Config(&handlerExtiConfig0);


	handlerExInData.pGPIOx 							   = GPIOA;
	handlerExInData.GPIO_PinConfig.GPIO_PinNumber      = PIN_1;
	handlerExInData.GPIO_PinConfig.GPIO_PinAltFunMode  = AF0;
	handlerExInData.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_IN;
	handlerExInData.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerExInData.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerExInData.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerExInData);


	//////////////////////////////-------------------------------------///////////////////////////////////////

	///////////////////////////------Configuración del pin transmisor y el USART2------/////////////////////////

	/*
	 * Se configuró el Pin A2 en función alternativa para USART2 en modo TX, sin interrupciones de recepcion.
	 */

	handlerPinTx.pGPIOx 							= GPIOA;
	handlerPinTx.GPIO_PinConfig.GPIO_PinNumber		= PIN_2;
	handlerPinTx.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinTx.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerPinTx.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEEDR_FAST;
	handlerPinTx.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinTx.GPIO_PinConfig.GPIO_PinAltFunMode	= AF7;
	GPIO_Config(&handlerPinTx);

	handlerUSART2.ptrUSARTx = USART2;
	handlerUSART2.USART_Config.USART_baudrate        = USART_BAUDRATE_115200;
	handlerUSART2.USART_Config.USART_datasize        = USART_DATASIZE_8BIT;
	handlerUSART2.USART_Config.USART_mode            = USART_MODE_TX;
	handlerUSART2.USART_Config.USART_stopbits        = USART_STOPBIT_1;
	handlerUSART2.USART_Config.USART_parity          = USART_PARITY_NONE;
	handlerUSART2.USART_Config.USART_enableInRx      = USART_INTERRUPT_RX_DISABLE;
	USART_Config(&handlerUSART2);

	//////////////////////////////-------------------------------------///////////////////////////////////////

	////////////-------------Configuración de pines para transistores---------------//////////////////

	/*
	 * Configuración de los PA6 y PA7 como salidas simples para swichear los 2
	 * transistores PNP 2N2907, los cuales permiten alimentar los 7 segmentos.
	 * Comenzamos sus estados logicos de forma de que uno tenga el estado opuesto
	 * al otro, esto con el fin de que luego se swichearan a una velocidad tal
	 * que podamos ver ambos segmentos ensendidos al tiempo. Esto se hace con el Timer 5 y sus interrupciones.
	 */


	handlerPinTran1.pGPIOx                              = GPIOA;
	handlerPinTran1.GPIO_PinConfig.GPIO_PinNumber       = PIN_6;
	handlerPinTran1.GPIO_PinConfig.GPIO_PinAltFunMode   = AF0;
	handlerPinTran1.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;
	handlerPinTran1.GPIO_PinConfig.GPIO_PinOPType       =  GPIO_OTYPE_PUSHPULL;
	handlerPinTran1.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PUPDR_NOTHING;
	handlerPinTran1.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinTran1);
	GPIO_WritePin(&handlerPinTran1, RESET);

	handlerPinTran2.pGPIOx                              = GPIOA;
	handlerPinTran2.GPIO_PinConfig.GPIO_PinNumber       = PIN_7;
	handlerPinTran2.GPIO_PinConfig.GPIO_PinAltFunMode   = AF0;
	handlerPinTran2.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;
	handlerPinTran2.GPIO_PinConfig.GPIO_PinOPType       =  GPIO_OTYPE_PUSHPULL;
	handlerPinTran2.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PUPDR_NOTHING;
	handlerPinTran2.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinTran2);
	GPIO_WritePin(&handlerPinTran2, SET);


	handlerTIM5.ptrTIMx = TIM5;
	handlerTIM5.TIMx_Config.TIMx_mode             = BTIMER_MODE_UP;
	handlerTIM5.TIMx_Config.TIMx_speed            = BTIMER_SPEED_100us;
	handlerTIM5.TIMx_Config.TIMx_period           = 100;
	handlerTIM5.TIMx_Config.TIMx_interruptEnable  = 1;
	BasicTimer_Config(&handlerTIM5);

	startTimer (&handlerTIM5);

	//////////////////////////////-------------------------------------///////////////////////////////////////

	/////////-------Configuración de pines para encender los segmentos del display--------////////////

	/*
	 * Tenemos 7 segmentos (LEDs) que prender, por lo que usaremos 7 pines, exatamente los C[1:7],
	 * cada uno como una salida simple en el cual su estado hara que se enciendan o se apaguen los
	 * LEDs. Como nuestro 7 segmentos es anodo comun, entonces el estado 0 de nuestros pines haran
	 * que se enciendan nuestros LEDs, y nuestro estado 1 lo contrario.
	 */


	handlerPinSeg1.pGPIOx                               = GPIOC;
	handlerPinSeg1.GPIO_PinConfig.GPIO_PinNumber        = PIN_1;
	handlerPinSeg1.GPIO_PinConfig.GPIO_PinAltFunMode    = AF0;
	handlerPinSeg1.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_OUT;
	handlerPinSeg1.GPIO_PinConfig.GPIO_PinOPType        =  GPIO_OTYPE_PUSHPULL;
	handlerPinSeg1.GPIO_PinConfig.GPIO_PinPuPdControl   = GPIO_PUPDR_NOTHING;
	handlerPinSeg1.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinSeg1);
	GPIO_WritePin(&handlerPinSeg1, RESET);

	handlerPinSeg2.pGPIOx                               = GPIOC;
	handlerPinSeg2.GPIO_PinConfig.GPIO_PinNumber        = PIN_2;
	handlerPinSeg2.GPIO_PinConfig.GPIO_PinAltFunMode    = AF0;
	handlerPinSeg2.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_OUT;
	handlerPinSeg2.GPIO_PinConfig.GPIO_PinOPType        =  GPIO_OTYPE_PUSHPULL;
	handlerPinSeg2.GPIO_PinConfig.GPIO_PinPuPdControl   = GPIO_PUPDR_NOTHING;
	handlerPinSeg2.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinSeg2);
	GPIO_WritePin(&handlerPinSeg2, RESET);

	handlerPinSeg3.pGPIOx                               = GPIOC;
	handlerPinSeg3.GPIO_PinConfig.GPIO_PinNumber        = PIN_3;
	handlerPinSeg3.GPIO_PinConfig.GPIO_PinAltFunMode    = AF0;
	handlerPinSeg3.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_OUT;
	handlerPinSeg3.GPIO_PinConfig.GPIO_PinOPType        =  GPIO_OTYPE_PUSHPULL;
	handlerPinSeg3.GPIO_PinConfig.GPIO_PinPuPdControl   = GPIO_PUPDR_NOTHING;
	handlerPinSeg3.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinSeg3);
	GPIO_WritePin(&handlerPinSeg3, RESET);

	handlerPinSeg4.pGPIOx                               = GPIOC;
	handlerPinSeg4.GPIO_PinConfig.GPIO_PinNumber        = PIN_4;
	handlerPinSeg4.GPIO_PinConfig.GPIO_PinAltFunMode    = AF0;
	handlerPinSeg4.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_OUT;
	handlerPinSeg4.GPIO_PinConfig.GPIO_PinOPType        =  GPIO_OTYPE_PUSHPULL;
	handlerPinSeg4.GPIO_PinConfig.GPIO_PinPuPdControl   = GPIO_PUPDR_NOTHING;
	handlerPinSeg4.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinSeg4);
	GPIO_WritePin(&handlerPinSeg4, RESET);

	handlerPinSeg5.pGPIOx                               = GPIOC;
	handlerPinSeg5.GPIO_PinConfig.GPIO_PinNumber        = PIN_5;
	handlerPinSeg5.GPIO_PinConfig.GPIO_PinAltFunMode    = AF0;
	handlerPinSeg5.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_OUT;
	handlerPinSeg5.GPIO_PinConfig.GPIO_PinOPType        =  GPIO_OTYPE_PUSHPULL;
	handlerPinSeg5.GPIO_PinConfig.GPIO_PinPuPdControl   = GPIO_PUPDR_NOTHING;
	handlerPinSeg5.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinSeg5);
	GPIO_WritePin(&handlerPinSeg5, RESET);

	handlerPinSeg6.pGPIOx                               = GPIOC;
	handlerPinSeg6.GPIO_PinConfig.GPIO_PinNumber        = PIN_6;
	handlerPinSeg6.GPIO_PinConfig.GPIO_PinAltFunMode    = AF0;
	handlerPinSeg6.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_OUT;
	handlerPinSeg6.GPIO_PinConfig.GPIO_PinOPType        =  GPIO_OTYPE_PUSHPULL;
	handlerPinSeg6.GPIO_PinConfig.GPIO_PinPuPdControl   = GPIO_PUPDR_NOTHING;
	handlerPinSeg6.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinSeg6);
	GPIO_WritePin(&handlerPinSeg6, RESET);

	handlerPinSeg7.pGPIOx                               = GPIOC;
	handlerPinSeg7.GPIO_PinConfig.GPIO_PinNumber        = PIN_7;
	handlerPinSeg7.GPIO_PinConfig.GPIO_PinAltFunMode    = AF0;
	handlerPinSeg7.GPIO_PinConfig.GPIO_PinMode          = GPIO_MODE_OUT;
	handlerPinSeg7.GPIO_PinConfig.GPIO_PinOPType        =  GPIO_OTYPE_PUSHPULL;
	handlerPinSeg7.GPIO_PinConfig.GPIO_PinPuPdControl   = GPIO_PUPDR_NOTHING;
	handlerPinSeg7.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEEDR_FAST;
	GPIO_Config(&handlerPinSeg7);
	GPIO_WritePin(&handlerPinSeg7, RESET);

	//////////////////////////////-------------------------------------///////////////////////////////////////

	//////////////--------------Configuración del botón regguetonero---------////////////////////

	/*
	 * Se configura el PIN B1 para leer flancos de subida con el EXTI 1, el
	 * boton se encuentra en configuracion PULLDOWN con tierra y resistencia externa.
	 */

	handlerExInButtonPin.pGPIOx 							= GPIOB;
	handlerExInButtonPin.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF0;
	handlerExInButtonPin.GPIO_PinConfig.GPIO_PinMode	    = GPIO_MODE_IN;
	handlerExInButtonPin.GPIO_PinConfig.GPIO_PinNumber      = PIN_1;
	handlerExInButtonPin.GPIO_PinConfig.GPIO_PinOPType      = GPIO_OTYPE_PUSHPULL;
	handlerExInButtonPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
	handlerExInButtonPin.GPIO_PinConfig.GPIO_PinSpeed       =  GPIO_OSPEEDR_MEDIUM;
	handlerExtiConfig1.pGPIOHandler                         = &handlerExInButtonPin;
	handlerExtiConfig1.edgeType                             = EXTERNAL_INTERRUPT_RISING_EDGE;
	extInt_Config(&handlerExtiConfig1);


	//////////////////////////////-------------------------------------///////////////////////////////////////

}

// A continuación se muestran todas las interrupciones (Callbacks) usados, solo se usaron interrupciones
//de Timers y de EXTI.

//PUNTO 2
// callback_extInt0() cambia el estado de clk y hace una lectura de data para posteriomente en el
//main saber si se dio una vuleta CW o CCW.
void callback_extInt0(void){
		clk = 1;
		data = GPIO_ReadPin(&handlerExInData);
}


// callback_extInt1() cambia el estado de msgButton para saber que hubo una interrupción por
//flanco de subida gracias al boton externo.
void callback_extInt1(void){
		msgButton = 1;
}


// BasicTimer2_Callback() cambia el estado del pin A5 para hacer el blinky
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyLed);
}

//BasicTimer5_Callback() cambia el estado lógico de los pines A6 y A7 para swichear los transistores que
//encienden y apagan una columna de segmentos.
void BasicTimer5_Callback(void){
	GPIOxTooglePin(&handlerPinTran1);
	GPIOxTooglePin(&handlerPinTran2);
	// Este es un pequeño error, al hacerlo así se genera un pequeño fantasma en los numeros que se presentan
	// La idea es que debes apagar los display, cambiar el numero y volver a enceder.
}
