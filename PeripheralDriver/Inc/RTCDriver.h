/*
 * RTCDriver.h
 *
 *  Created on: 5/11/2022
 *      Author: ALEJANDRA MARIA
 */

#ifndef RTCDRIVER_H_
#define RTCDRIVER_H_

#include "stm32f4xx.h"

#endif /* RTCDRIVER_H_ */


#define MONDAY   1
#define THUESDAY 2
#define WENESDAY 3
#define THURSDAY 4
#define FRIDAY   5
#define SATURDAY 6
#define SUNDAY   7


typedef struct
{
	uint8_t rtcYear;
	uint8_t rtcMonth;
	uint8_t rtcWeekDay;
	uint16_t rtcDay;
	uint8_t rtcHours;
	uint8_t rtcMinutes;
	uint8_t rtcSeconds;

}RTC_Config_t;

typedef struct
{
	RTC_TypeDef *ptrRTC;
	RTC_Config_t RTC_config;
}RTC_Handler_t;


void Rtc_Congif (RTC_Handler_t *ptrHandlerRtc);
void Rtc_BDC_Convertion (RTC_Handler_t *ptrHandlerRtc);
uint8_t *getDate(void);

