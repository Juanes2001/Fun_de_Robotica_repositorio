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


#define MCO1 0
#define MCO2 1

#define RCC_20MHz  0
#define RCC_30MHz  1
#define RCC_40MHz  2
#define RCC_50MHz  3
#define RCC_60MHz  4
#define RCC_70MHz  5
#define RCC_80MHz  6
#define RCC_90MHz  7
#define RCC_100MHz 8



void RCC_enableMaxFrequencies(uint8_t frequency);
void show_MaxFreq (uint8_t outputType, uint8_t div);



#endif /* RCCHUNMHZ_H_ */
