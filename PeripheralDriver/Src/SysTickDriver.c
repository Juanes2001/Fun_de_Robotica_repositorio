/*
 * SysTickDriver.c
 *
 *  Created on: Dic 1, 2022
 *      Author: Juan Esteban Rodriguez
 *
 * Este driver controla el timer que trae por defecto todo procesador ARM Cortex Mx,
 * el cual hace parte del sistema independiente de la empresa fabricaten del MCU.
 *
 * Para encontrar cual es su registro de configuracion, debemos utilizar el manual
 * generico del procesador Cortex-M4, ya que es alli donde se encuentra la documentacion
 * para el periferico
 *
 * En el archivo core_cm4.h, la estructura que define el periferico se llama SysTick_Type
 *
 */

#include "SysTickDriver.h"

#define SYSTICK_LOAD_VALUE		1600000	//numero de ciclos en 100ms.

uint64_t ticks;

/**/
void config_SysTicksMs(void){
	//Reiniciamos el valor de la variable que cuenta tiempo
	ticks = 0;

	//Cargando el valor del limite de incrementos que representan 1ms
	SysTick->LOAD = SYSTICK_LOAD_VALUE-1;

	//Limpiamos el valor actual del Systick
	SysTick->VAL = 0;

	//Configuramos el reloj interno como el reloj para el timer
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

//	//Desactivamos las interrupciones globales
//	__disable_irq();
//
//	//Activamos la interrupcion debida al conteo a cero del SysTick
//	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
//
//	//Matriculamos la interrupcion en el NVIC
//	__NVIC_EnableIRQ(SysTick_IRQn);
//
//	//Activamos de nuevo las interrupciones globales
//	__enable_irq();

}


void delay_Ms (uint32_t Ms){
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;

	for(int i= Ms ; i>0 ; i-= 100){

		while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)){}

	}
	SysTick->CTRL &=  ~SysTick_CTRL_ENABLE_Msk;
}

