/*
 * RCCHunMHz.c
 *
 *  Created on: 13 nov. 2022
 *      Author: JUAN ESTEBAN
 */

#include "stm32f4xx.h"
#include "RCCHunMHz.h"
#include "GPIOxDriver.h"



void RCC_enableMaxFrequencies(uint8_t frequency){

	//Nos aseguramos que el PLL esta apagado
	RCC->CR &= ~(RCC_CR_PLLON);
	//Activamos el PWR parapoder activar el uso de 100MHz de velocidad
	RCC->APB1ENR = RCC_APB1ENR_PWREN;
	//Le damos la opcion al PWR de permitir al MCU para correr una frecuencia de maximo 100MHz
	PWR->CR |= (0b11 << 14);


	//Antes de configurar el PLL referenciamos cual sera la fuente para el PLL, en nuestro caso sera el HSI sobre el mismo
	//registro
	RCC->PLLCFGR &= ~(0b1 << 22);

	// Aqui si dependiendo de la velocidad que queramos usaremos unos multiplicadores y divisores especificos.
	// La formula es freqSys = freq input * (pllN/(pllM*pllP))

	switch (frequency) {
		case RCC_20MHz:{
			//freqsys = 16MHz * (120/(16*6)) =  20MHz
			RCC->PLLCFGR &= ~(0xFF << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR |= (16 << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR &= ~(0x1FF << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR |= (120 << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR &= ~(0b11 << RCC_PLLCFGR_PLLP_Pos);
			RCC->PLLCFGR |= (0b10 << RCC_PLLCFGR_PLLP_Pos); // Division por 6 en el pllP


			//Seleccionamos los preescalers adecuados para los AHB y APBx en el registro RCC_CFGR No se divide nada en este caso
			RCC->CFGR &= ~(0b1000 << 4); // Division del AHB por 1
			RCC->CFGR &= ~(0b100 << 13); // division del APB2 por 1
			RCC->CFGR &= ~(0b100 << 10); // Division del APB1 por 1

			//Configurado el PLL para salida de 100MHz entonces ya podemos activar el PLL
			RCC->CR |= RCC_CR_PLLON;
			while (!(RCC->CR & RCC_CR_PLLRDY)){
				__NOP();
			}

			//Antes de configurar el PLL del RCC, debemos cambiar la velocidad de lectura de la memoria flash
			FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY_Pos);
			FLASH->ACR |= (0b000 << FLASH_ACR_LATENCY_Pos);

			break;
		}case RCC_30MHz:{
			//freqsys = 16MHz * (120/(16*4)) =  30MHz
			RCC->PLLCFGR &= ~(0xFF << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR |= (16 << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR &= ~(0x1FF << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR |= (120 << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR &= ~(0b11 << RCC_PLLCFGR_PLLP_Pos);
			RCC->PLLCFGR |= (0b01 << RCC_PLLCFGR_PLLP_Pos); // Division por 4 en el pllP

			//Seleccionamos los preescalers adecuados para los AHB y APBx en el registro RCC_CFGR No se divide nada en este caso
			RCC->CFGR &= ~(0b1000 << 4); // Division del AHB por 1
			RCC->CFGR &= ~(0b100 << 13); // division del APB2 por 1
			RCC->CFGR &= ~(0b100 << 10); // Division del APB1 por 1

			//Configurado el PLL para salida de 100MHz entonces ya podemos activar el PLL
			RCC->CR |= RCC_CR_PLLON;
			while (!(RCC->CR & RCC_CR_PLLRDY)){
				__NOP();
			}

			//Antes de configurar el PLL del RCC, debemos cambiar la velocidad de lectura de la memoria flash
			FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY_Pos);
			FLASH->ACR |= (0b000 << FLASH_ACR_LATENCY_Pos);

			break;
		}case RCC_40MHz:{
			//freqsys = 16MHz * (240/(16*6)) =  40MHz
			RCC->PLLCFGR &= ~(0xFF << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR |= (16 << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR &= ~(0x1FF << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR |= (240 << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR &= ~(0b11 << RCC_PLLCFGR_PLLP_Pos);
			RCC->PLLCFGR |= (0b10 << RCC_PLLCFGR_PLLP_Pos); // Division por 6 en el pllP

			//Seleccionamos los preescalers adecuados para los AHB y APBx en el registro RCC_CFGR No se divide nada en este caso
			RCC->CFGR &= ~(0b1000 << 4); // Division del AHB por 1
			RCC->CFGR &= ~(0b100 << 13); // division del APB2 por 1
			RCC->CFGR &= ~(0b100 << 10); // Division del APB1 por 1

			//Configurado el PLL para salida de 100MHz entonces ya podemos activar el PLL
			RCC->CR |= RCC_CR_PLLON;
			while (!(RCC->CR & RCC_CR_PLLRDY)){
				__NOP();
			}

			//Antes de configurar el PLL del RCC, debemos cambiar la velocidad de lectura de la memoria flash
			FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY_Pos);
			FLASH->ACR |= (0b001 << FLASH_ACR_LATENCY_Pos);


			break;
		}case RCC_50MHz:{
			//freqsys = 16MHz * (100/(16*2)) =  50MHz
			RCC->PLLCFGR &= ~(0xFF << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR |= (16 << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR &= ~(0x1FF << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR |= (100 << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR &= ~(0b11 << RCC_PLLCFGR_PLLP_Pos);
			RCC->PLLCFGR |= (0b00 << RCC_PLLCFGR_PLLP_Pos); // Division por 2 en el pllP

			//Seleccionamos los preescalers adecuados para los AHB y APBx en el registro RCC_CFGR No se divide nada en este caso
			RCC->CFGR &= ~(0b1000 << 4); // Division del AHB por 1
			RCC->CFGR &= ~(0b100 << 13); // division del APB2 por 1
			RCC->CFGR &= ~(0b100 << 10); // Division del APB1 por 1

			//Configurado el PLL para salida de 100MHz entonces ya podemos activar el PLL
			RCC->CR |= RCC_CR_PLLON;
			while (!(RCC->CR & RCC_CR_PLLRDY)){
				__NOP();
			}

			//Antes de configurar el PLL del RCC, debemos cambiar la velocidad de lectura de la memoria flash
			FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY_Pos);
			FLASH->ACR |= (0b001 << FLASH_ACR_LATENCY_Pos);

			break;
		}case RCC_60MHz:{
			//freqsys = 16MHz * (120/(16*2)) =  60MHz
			RCC->PLLCFGR &= ~(0xFF << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR |= (16 << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR &= ~(0x1FF << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR |= (120 << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR &= ~(0b11 << RCC_PLLCFGR_PLLP_Pos);
			RCC->PLLCFGR |= (0b00 << RCC_PLLCFGR_PLLP_Pos); // Division por 2 en el pllP

			//Seleccionamos los preescalers adecuados para los AHB y APBx en el registro RCC_CFGR
			RCC->CFGR &= ~(0b1000 << 4); // Division del AHB por 1
			RCC->CFGR &= ~(0b100 << 13); // division del APB2 por 1
			RCC->CFGR |= (0b100 << 10); // Division del APB1 por 2

			//Configurado el PLL para salida de 100MHz entonces ya podemos activar el PLL
			RCC->CR |= RCC_CR_PLLON;
			while (!(RCC->CR & RCC_CR_PLLRDY)){
				__NOP();
			}

			//Antes de configurar el PLL del RCC, debemos cambiar la velocidad de lectura de la memoria flash
			FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY_Pos);
			FLASH->ACR |= (0b001 << FLASH_ACR_LATENCY_Pos);

			break;
		}case RCC_70MHz:{
			//freqsys = 16MHz * (140/(16*2)) =  70MHz
			RCC->PLLCFGR &= ~(0xFF << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR |= (16 << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR &= ~(0x1FF << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR |= (140 << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR &= ~(0b11 << RCC_PLLCFGR_PLLP_Pos);
			RCC->PLLCFGR |= (0b00 << RCC_PLLCFGR_PLLP_Pos); // Division por 2 en el pllP

			//Seleccionamos los preescalers adecuados para los AHB y APBx en el registro RCC_CFGR
			RCC->CFGR &= ~(0b1000 << 4); // Division del AHB por 1
			RCC->CFGR &= ~(0b100 << 13); // division del APB2 por 1
			RCC->CFGR |= (0b100 << 10); // Division del APB1 por 2

			//Configurado el PLL para salida de 100MHz entonces ya podemos activar el PLL
			RCC->CR |= RCC_CR_PLLON;
			while (!(RCC->CR & RCC_CR_PLLRDY)){
				__NOP();
			}

			//Antes de configurar el PLL del RCC, debemos cambiar la velocidad de lectura de la memoria flash
			FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY_Pos);
			FLASH->ACR |= (0b010 << FLASH_ACR_LATENCY_Pos);

			break;
		}case RCC_80MHz:{
			//freqsys = 16MHz * (160/(16*2)) =  80MHz
			RCC->PLLCFGR &= ~(0xFF << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR |= (16 << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR &= ~(0x1FF << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR |= (160 << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR &= ~(0b11 << RCC_PLLCFGR_PLLP_Pos);
			RCC->PLLCFGR |= (0b00 << RCC_PLLCFGR_PLLP_Pos); // Division por 2 en el pllP

			//Seleccionamos los preescalers adecuados para los AHB y APBx en el registro RCC_CFGR
			RCC->CFGR &= ~(0b1000 << 4); // Division del AHB por 1
			RCC->CFGR &= ~(0b100 << 13); // division del APB2 por 1
			RCC->CFGR |= (0b100 << 10); // Division del APB1 por 2

			//Configurado el PLL para salida de 100MHz entonces ya podemos activar el PLL
			RCC->CR |= RCC_CR_PLLON;
			while (!(RCC->CR & RCC_CR_PLLRDY)){
				__NOP();
			}

			//Antes de configurar el PLL del RCC, debemos cambiar la velocidad de lectura de la memoria flash
			FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY_Pos);
			FLASH->ACR |= (0b010 << FLASH_ACR_LATENCY_Pos);

			break;
		}case RCC_90MHz:{
			//freqsys = 16MHz * (180/(16*2)) =  90MHz
			RCC->PLLCFGR &= ~(0xFF << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR |= (16 << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR &= ~(0x1FF << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR |= (180 << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR &= ~(0b11 << RCC_PLLCFGR_PLLP_Pos);
			RCC->PLLCFGR |= (0b00 << RCC_PLLCFGR_PLLP_Pos); // Division por 2 en el pllP

			//Seleccionamos los preescalers adecuados para los AHB y APBx en el registro RCC_CFGR
			RCC->CFGR &= ~(0b1000 << 4); // Division del AHB por 1
			RCC->CFGR &= ~(0b100 << 13); // division del APB2 por 1
			RCC->CFGR |= (0b100 << 10); // Division del APB1 por 2

			//Configurado el PLL para salida de 100MHz entonces ya podemos activar el PLL
			RCC->CR |= RCC_CR_PLLON;
			while (!(RCC->CR & RCC_CR_PLLRDY)){
				__NOP();
			}

			//Antes de configurar el PLL del RCC, debemos cambiar la velocidad de lectura de la memoria flash
			FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY_Pos);
			FLASH->ACR |= (0b010 << FLASH_ACR_LATENCY_Pos);

			break;
		}case RCC_100MHz:{
			//freqsys = 16MHz * (100/(8*2)) =  100MHz
			RCC->PLLCFGR &= ~(0xFF << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos);
			RCC->PLLCFGR &= ~(0x1FF << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR |= (100 << RCC_PLLCFGR_PLLN_Pos);
			RCC->PLLCFGR &= ~(0b11 << RCC_PLLCFGR_PLLP_Pos);
			RCC->PLLCFGR |= (0b00 << RCC_PLLCFGR_PLLP_Pos); // Division por 2 en el pllP

			//Seleccionamos los preescalers adecuados para los AHB y APBx en el registro RCC_CFGR
			RCC->CFGR &= ~(0b1000 << 4); // Division del AHB por 1
			RCC->CFGR &= ~(0b100 << 13); // division del APB2 por 1
			RCC->CFGR |= (0b100 << 10); // Division del APB1 por 2

			//Configurado el PLL para salida de 100MHz entonces ya podemos activar el PLL
			RCC->CR |= RCC_CR_PLLON;
			while (!(RCC->CR & RCC_CR_PLLRDY)){
				__NOP();
			}

			//Antes de configurar el PLL del RCC, debemos cambiar la velocidad de lectura de la memoria flash
			FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY_Pos);
			FLASH->ACR |= (0b011 << FLASH_ACR_LATENCY_Pos);

			break;
		}
		default:{
			break;
		}
	}


	// Se configura como system clock al PLL
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)){
		__NOP();
	}

//	SystemCoreClockUpdate();

}


void show_MaxFreq (uint8_t outputType ,uint8_t div){

	// Vemos si se quiere sacar el valor por el MCO1 o el MCO2

	if (outputType  == MCO1){

		//Configuramos la salida MCO2 para verificar el estado real del MCU y su velocidad
		switch (div) {
			case 1:{
				// No dividimos el clock
				RCC->CFGR &= ~(0b100 << 24);
				break;
			}case 2:{
				// dividimos el clock a la mitad
				RCC->CFGR &= ~(0b111  << 24);
				RCC->CFGR |=  (0b100  << 24);

				break;
			}case 3:{
				// dividimos el clock 3 veces
				RCC->CFGR &= ~(0b111  << 24);
				RCC->CFGR |=  (0b101  << 24);
				break;
			}case 4:{
				// Dividimos el Clock 4 veces
				RCC->CFGR &= ~(0b111  << 24);
				RCC->CFGR |=  (0b110  << 24);
				break;
			}case 5:{
				// Divimos el clock 5 veces
				RCC->CFGR &= ~(0b111  << 24);
				RCC->CFGR |=  (0b111  << 24);
				break;
			}
			default:{
				break;
			}
		}
		// habilitamos la salida del MCO1 para el PLL clock
		RCC->CFGR |= (0b11 << 21);
	//	RCC->CFGR |= RCC_CFGR_MCO2;



	}else{

		//Configuramos la salida MCO2 para verificar el estado real del MCU y su velocidad
		switch (div) {
			case 1:{
				// No dividimos el clock
				RCC->CFGR &= ~(0b100 << 27);
				break;
			}case 2:{
				// dividimos el clock a la mitad
				RCC->CFGR &= ~(0b111 << 27);
				RCC->CFGR |=  (0b100  << 27);

				break;
			}case 3:{
				// dividimos el clock 3 veces
				RCC->CFGR &= ~(0b111 << 27);
				RCC->CFGR |=  (0b101  << 27);
				break;
			}case 4:{
				// Dividimos el Clock 4 veces
				RCC->CFGR &= ~(0b111 << 27);
				RCC->CFGR |=  (0b110  << 27);
				break;
			}case 5:{
				// Divimos el clock 5 veces
				RCC->CFGR &= ~(0b111 << 27);
				RCC->CFGR |=  (0b111  << 27);
				break;
			}
			default:{
				break;
			}
		}
		// habilitamos la salida del MCO2 para el PLL clock
		RCC->CFGR |= (0b11 << 30);
	//	RCC->CFGR |= RCC_CFGR_MCO2;

	}

}




