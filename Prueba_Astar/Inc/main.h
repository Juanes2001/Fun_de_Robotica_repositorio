/*
 * main.h
 *
 *  Created on: Aug 23, 2024
 *      Author: juan
 */

#ifndef MAIN_H_
#define MAIN_H_

// Includes
#include "stm32f4xx.h"
#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "USARTxDriver.h"
#include "RCCHunMHz.h"
#include "Astar.h"

extern void inSystem (void);
extern void parseCommands(char *stringVector);
extern void builtTerminalString (char** terminalString);



//Definición Handlers
//GPIO
//Pin del User blinky
extern GPIO_Handler_t handlerPinA5;

//Pines de comunicacion USART
extern GPIO_Handler_t handlerPinRx;
extern GPIO_Handler_t handlerPinTx;

//Pin para visualizar la velocidad del micro
extern GPIO_Handler_t handlerMCO2Show;

//Timers
extern BasicTimer_Handler_t handlerTimerBlinky; // Timer 3

//Usart
extern USART_Handler_t handlerUSART;
extern USART_Handler_t handlerUSART;

// Astar
extern AStar_distancesHandler handlerAstarParameters;
extern costChangesAndPos_t handlerCostsAstar;

// Variables para los comandos
extern char bufferReception[64];
extern uint8_t counterReception;
extern uint8_t doneTransaction;
extern uint8_t rxData;
extern char cmd[32];
extern unsigned int firstParameter;
extern unsigned int secondParameter;
extern unsigned int thirdParameter;
extern char userMsg[64];
extern char bufferMsg[64];

// Definición de la matriz de string que almacenará
extern char stringMatrix[10][10];
extern uint8_t stringColumn;
extern uint8_t stringRow;

// Banderas
extern uint8_t flagAstar;
extern uint8_t starWorking;

//Mensajes
extern const char* msg_NotWorking;
extern const char* msg_InsertGrid;



#endif /* MAIN_H_ */
