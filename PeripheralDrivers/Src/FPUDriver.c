/*
 * FPUDriver.c
 *
 *  Created on: 1 dic. 2022
 *      Author: JUAN ESTEBAN
 */


#include "FPUDriver.h"
#include "core_cm4.h"
#define SBD_CPACR_FULL	(0xF << 20)

void config_FPU(void){
	//Registro que activa el periférico del FPU en full acess
	SCB->CPACR |= SBD_CPACR_FULL;
}
