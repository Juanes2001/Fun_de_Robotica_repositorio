/*
 * RCCHunMHz.c
 *
 *  Created on: 13 nov. 2022
 *      Author: JUAN ESTEBAN
 */

#include "stm32f4xx.h"
#include "RCCHunMHz.h"
#include "GPIOxDriver.h"



void RCC_enableMaxFrequencies(void){


	//Nos aseguramos que el PLL esta apagado para asi hacer la configuracion del mismo, ademas selecc
	RCC->CR &= ~(RCC_CR_PLLON);
	//Activamos el PWR parapoder activar el uso de 100MHz de velocidad
	RCC->APB1ENR = RCC_APB1ENR_PWREN;
	//Le damos la opcion al PWR de permitir al MCU para correr una frecuencia de 100MHz
	PWR->CR |= (0b11 << 14);


	//Antes de configurar el PLL referenciamos cual sera la fuente para el PLL, en nuestro caso sera el HSI sobre el mismo
	//registro
	RCC->PLLCFGR &= 0;

	//Montamos sobre el RCC_PLL config las subdivisiones necesarias para obtener la salida de frecuencia de reloj que deseamos
	RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos);
	RCC->PLLCFGR |= (100 << RCC_PLLCFGR_PLLN_Pos);
	RCC->PLLCFGR &= ~(0b11 << RCC_PLLCFGR_PLLP_Pos);

	//Configurado el PLL para salida de 100MHz entonces ya podemos activar el PLL
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY)){
		__NOP();
	}

	//Seleccionamos los preescalers adecuados para los AHB y APBx en el registro RCC_CFGR
	RCC->CFGR &= ~(0b111 << 13);
	RCC->CFGR |= (0b100 << 10);
	RCC->CFGR &= ~(0xF << 4);

	//Antes de configurar el PLL del RCC, debemos cambiar la velocidad de lectura de la memoria flash
	FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY_Pos);
	FLASH->ACR |= (0b011 << FLASH_ACR_LATENCY_Pos);


	// Se configura como system clock al PLL
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)){
		__NOP();
	}



}

void RCC_disableMaxFrequencies(void){

	//Nos aseguramos que el PLL esta apagado
	RCC->CR &= ~(RCC_CR_PLLON);
	//Activamos el PWR parapoder activar el uso de 16MHz de velocidad
	RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;
	//Le damos la opcion al PWR de permitir al MCU para correr una frecuencia de 16MHz
	PWR->CR &= ~(0b01 << 14);

	//Antes de configurar el PLL del RCC, debemos cambiar la velocidad de lectura de la memoria flash
	FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY_Pos);

	// Se configura como system clock al HSI
	RCC->CFGR &= ~(0b11 << 0);

	while(!(RCC->CFGR & RCC_CFGR_SWS_HSI)){
		__NOP();
	}

}


void show_MaxFreq (void){

	//Le damos una division a la señal que pasa por MCO2 de 5 para poder leerla en el osciloscopio, pin C9
	RCC->CFGR &= ~(0b111 << 27);
	RCC->CFGR |= (0b000<< 27);

	//Configuramos la salida MCO2 para verificar el estado real del MCU y su velocidad
	RCC->CFGR &= ~(0b11 << 30);
	RCC->CFGR |= RCC_CFGR_MCO2;

}

void show_StandartFreq (void){
	//Le damos una division a la señal que pasa por MCO2 de 5 para poder leerla en el osciloscopio, pin C9
	RCC->CFGR &= ~(0b111 << 27);
	RCC->CFGR |= (0b000<< 27);

	//Configuramos la salida MCO2 para verificar el estado real del MCU y su velocidad
	RCC->CFGR &= ~(0b11 << 30);


}




