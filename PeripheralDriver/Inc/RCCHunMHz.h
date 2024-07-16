/*
 * RCCHunMHz.h
 *
 *  Created on: 21/11/2022
 *      Author: ALEJANDRA MARIA
 */

#ifndef RCCHUNMHZ_H_
#define RCCHUNMHZ_H_

#include "stm32f4xx.h"
#include "GPIOxDriver.h"



void RCC_enableMaxFrequencies(void);
void RCC_disableMaxFrequencies(void);
void show_MaxFreq (void);
void show_StandartFreq (void);



#endif /* RCCHUNMHZ_H_ */
