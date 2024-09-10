/*
 * SysTickDriver.h
 *
 *  Created on: Dic 1, 2022
 *      Author: Juan Esteban Rodriguez
 */

#ifndef SYSTICKDRIVER_H_
#define SYSTICKDRIVER_H_

#include <stm32f4xx.h>

void config_SysTicksMs(void);
void delay_Ms (uint32_t Ms);


#endif /* SYSTICKDRIVER_H_ */
