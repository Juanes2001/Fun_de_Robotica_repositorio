/*
 * GPIOxDriver.h
 *
 *  Created on: Aug 27, 2022
 *      Author: JUAN ESTEBAN
 *
 *      Este proghrama se encargara de las definiciones relacionadas unicamente con el
 *      debido manejo y control del periferico GPIOx (funciones del controlador)
 */

#ifndef GPIOXDRIVER_H_
#define GPIOXDRIVER_H_

// Incluyendo este archivo estamos incluyendo tambien el correspondiente al GPIOx
#include "stm32f4xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;        //PIN con el que deseamos trabajar
	uint8_t GPIO_PinMode;          // Modo de la configuracion: entrada, salida, analogo, f, alternativa
	uint8_t GPIO_PinSpeed;         // Velocidad de respuesta del PIN (solo para digital)
	uint8_t GPIO_PinPuPdControl;    // Seleccionamos si deseamos una salida Pull-up, Pull-down o "libre"
	uint8_t GPIO_PinOPType;        // Trabaja de la mano con el anterior, selecciona salida PUPD o OpenDrain
	uint8_t GPIO_PinAltFunMode;    // Selecciona cual es la funcion alternativa que se esta configurando


} GPIO_PinConfig_t;

/*
 * Esta es una estructura que contiene dos elementos:
 * . La direccion del puerto que se esta utilizando
 * . La configuracion especifica del PIN que se esta utilizando
 */

typedef struct
{
	GPIO_TypeDef    *pGPIOx;        //Direccion del puerto al que el PIN corresponde
	GPIO_PinConfig_t  GPIO_PinConfig; // Configuracion del PIN
}GPIO_Handler_t;

//Definicion de las cabeceras de las funciones el GPIOxDriver
void GPIO_Config (GPIO_Handler_t *pGPIOHandler);
void GPIO_WritePin (GPIO_Handler_t *pPinHandler, uint8_t newState);
void GPIO_WritePin_Afopt (GPIO_Handler_t *pPinHandler, uint8_t newState);
uint32_t GPIO_ReadPin (GPIO_Handler_t *pPinHandler);
void GPIOxTooglePin (GPIO_Handler_t *pPinState);


#define GPIO_MODE_IN        0
#define GPIO_MODE_OUT       1
#define GPIO_MODE_ALTFN     2
#define GPIO_MODE_ANALOG    3

/*8.4.2 GPIOx OTYPER (un bit por PIN)
 */

#define GPIO_OTYPE_PUSHPULL     0
#define GPIO_OTYPE_OPENDRAIN    1

/* 8.4.3 GPIOx_OSPEEDR (dos bits por cada PIN)
 */

#define GPIO_OSPEEDR_LOW      0
#define GPIO_OSPEEDR_MEDIUM   1
#define GPIO_OSPEEDR_FAST     2
#define GPIO_OSPEEDR_HIGH     3

/* 8.4.4 GPIOx_PUDER (dos bit por cada PIN)
 */

#define GPIO_PUPDR_NOTHING     0
#define GPIO_PUPDR_PULLUP      1
#define GPIO_PUPDR_PULLDOWN    2
#define GPIO_PUPDR_RESERVED    3

/* 8.4.5 GPIOx_IDR (un bit por PIN) - este es el registro para leer el estado de un PIN
 */

/* 8.4.6 GPIOx_ODR (un bit por PIN) - este es el registro para escribir el estado de un
 * PIN (1 o 0). Este registro puede ser escrito o leido desde el software, pero no garantiza
 * una escritura "atomica", por lo cual es preferible utilizar el registro BSRR
 */

/*
 * Definicion de los nombres de los pines
 */

#define PIN_0                0
#define PIN_1                1
#define PIN_2                2
#define PIN_3                3
#define PIN_4                4
#define PIN_5                5
#define PIN_6                6
#define PIN_7                7
#define PIN_8                8
#define PIN_9                9
#define PIN_10               10
#define PIN_11               11
#define PIN_12               12
#define PIN_13               13
#define PIN_14               14
#define PIN_15               15

/* Definicion de las funciones alternativas
 */

#define AF0  0b0000
#define AF1  0b0001
#define AF2  0b0010
#define AF3  0b0011
#define AF4  0b0100
#define AF5  0b0101
#define AF6  0b0110
#define AF7  0b0111
#define AF8  0b1000
#define AF9  0b1001
#define AF10 0b1010
#define AF11 0b1011
#define AF12 0b1100
#define AF13 0b1101
#define AF14 0b1110
#define AF15 0b1111




#endif /* GPIOXDRIVER_H_ */
