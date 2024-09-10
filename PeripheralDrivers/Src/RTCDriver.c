/*
 * RTCDriver.c
 *
 *  Created on: 5/11/2022
 *      Author: Juan Esteban Rodriguez Ochoa
 */


#include "RTCDriver.h"


void Rtc_Congif (RTC_Handler_t *ptrHandlerRtc){
	uint8_t HU  = 0;
	uint8_t HD  = 0;
	uint8_t MU  = 0;
	uint8_t MD  = 0;
	uint8_t SU  = 0;
	uint8_t SD  = 0;
	uint8_t MoU = 0;
	uint8_t MoD = 0;
	uint8_t DU  = 0;
	uint8_t DD  = 0;
	uint8_t YU  = 0;
	uint8_t YD  = 0;
	//Activamos el PWR parapoder desactivar su sistema de bloqueo en el RTC
	RCC->APB1ENR = RCC_APB1ENR_PWREN;
	//Desactivamos la proteccion de PWR para modificar registros
	PWR->CR |= PWR_CR_DBP;

	/* 1. Debemos activar el LSE */
	//Encendemos el LSE clock
	RCC->BDCR &= ~RCC_BDCR_LSEON;
	RCC->BDCR |= RCC_BDCR_LSEON;

	// Esperamos hasta que el LSE este listo para ser usado
	while(!(RCC->BDCR & RCC_BDCR_LSERDY)){
		__NOP();
	}
	// Seleccionamos el LSE para RTC
	RCC->BDCR &= ~(0b11 << RCC_BDCR_RTCSEL_Pos);
	RCC->BDCR |= (0b01 << RCC_BDCR_RTCSEL_Pos);
	// Activamos el RTC clock
	RCC->BDCR |= RCC_BDCR_RTCEN;

	// convertimos el formato de horas en BDC
	 HU = (ptrHandlerRtc->RTC_config.rtcHours%10);
	 HD = (ptrHandlerRtc->RTC_config.rtcHours-(ptrHandlerRtc->RTC_config.rtcHours%10))/10;
	 // convertimos el formato de Minutos en BDC
	 MU = (ptrHandlerRtc->RTC_config.rtcMinutes%10);
	 MD = (ptrHandlerRtc->RTC_config.rtcMinutes-(ptrHandlerRtc->RTC_config.rtcMinutes%10))/10;
	 // convertimos el formato de segundos en BDC
	 SU = (ptrHandlerRtc->RTC_config.rtcSeconds%10);
	 SD = (ptrHandlerRtc->RTC_config.rtcSeconds-(ptrHandlerRtc->RTC_config.rtcSeconds%10))/10;
	 // convertimos el formato de meses en BDC
	 MoU = (ptrHandlerRtc->RTC_config.rtcMonth%10);
	 MoD = (ptrHandlerRtc->RTC_config.rtcMonth-(ptrHandlerRtc->RTC_config.rtcMonth%10))/10;
	 // convertimos el formato de dias en BDC
	 DU = (ptrHandlerRtc->RTC_config.rtcDay%10);
	 DD = (ptrHandlerRtc->RTC_config.rtcDay-(ptrHandlerRtc->RTC_config.rtcDay%10))/10;
	 //Convertimos el formato de años en BBC
	 YU = (ptrHandlerRtc->RTC_config.rtcYear%10);
	 YD = (ptrHandlerRtc->RTC_config.rtcYear-(ptrHandlerRtc->RTC_config.rtcYear%10))/10;
	//Activamos el modo escritura de los registros RTC
	ptrHandlerRtc->ptrRTC->WPR =  0xCA;
	ptrHandlerRtc->ptrRTC->WPR =  0x53;

	//CONFIGURACION DEL CALENDARIO :
	//Seteamos el bit INIT para entrar en el initialization mode
	ptrHandlerRtc->ptrRTC->ISR |= RTC_ISR_INIT;
	// Y permitimos las actualizaciones del calendario
	ptrHandlerRtc->ptrRTC->ISR |= RTC_ISR_INITF;

	//Configuramos para que tengamos un reloj de un segundo de precision
	// Para ello ponemos PREDIV_A y PREDIV_S en RTC_PRER
	ptrHandlerRtc->ptrRTC->PRER |= (0x0FF << RTC_PRER_PREDIV_S_Pos);
	ptrHandlerRtc->ptrRTC->PRER |= (0x07F << RTC_PRER_PREDIV_A_Pos);

	//Sincronizamos el contador del calendario con los valores iniciales puestos en los registros de la hora
	ptrHandlerRtc->ptrRTC->CR |= RTC_CR_BYPSHAD;

	//Seteamos la hora, minutos y segundos
	//Seteamos el formato de 24 horas
	ptrHandlerRtc->ptrRTC->TR &= ~(RTC_TR_PM);

	ptrHandlerRtc->ptrRTC->TR &= ~(0xF << RTC_TR_SU_Pos);
	ptrHandlerRtc->ptrRTC->TR |= (SU << RTC_TR_SU_Pos);
	ptrHandlerRtc->ptrRTC->TR |= (SD << RTC_TR_ST_Pos);
	ptrHandlerRtc->ptrRTC->TR |= (MU << RTC_TR_MNU_Pos);
	ptrHandlerRtc->ptrRTC->TR |= (MD << RTC_TR_MNT_Pos);
	ptrHandlerRtc->ptrRTC->TR |= (HU << RTC_TR_HU_Pos);
	ptrHandlerRtc->ptrRTC->TR |= (HD << RTC_TR_HT_Pos);

	//Seteamos la fecha, dias, mes y año

	ptrHandlerRtc->ptrRTC->DR = 0;

	ptrHandlerRtc->ptrRTC->DR |= (DU << RTC_DR_DU_Pos);
	ptrHandlerRtc->ptrRTC->DR |= (DD << RTC_DR_DT_Pos);
	ptrHandlerRtc->ptrRTC->DR |= (MoU << RTC_DR_MU_Pos);
	ptrHandlerRtc->ptrRTC->DR |= (MoD << RTC_DR_MT_Pos);
	ptrHandlerRtc->ptrRTC->DR |= (ptrHandlerRtc->RTC_config.rtcWeekDay << RTC_DR_WDU_Pos);
	ptrHandlerRtc->ptrRTC->DR |= (YU << RTC_DR_YU_Pos);
	ptrHandlerRtc->ptrRTC->DR |= (YD << RTC_DR_YT_Pos);

	//Salimos del modo INIT de inicializacion
	RCC->BDCR |= RCC_BDCR_RTCEN;
	RTC->ISR &= ~RTC_ISR_INIT;
	PWR->CR &= ~ PWR_CR_DBP;

	RTC->CR &= ~RTC_CR_BYPSHAD;
	RTC->WPR = (0xFF); // Key Lock write protection



}

//void Rtc_BDC_Convertion (RTC_Handler_t *ptrHandlerRtc){
//	// convertimos el formato de horas en BDC
//	 HU = (ptrHandlerRtc->RTC_config.rtcHours%10);
//	 HD = (ptrHandlerRtc->RTC_config.rtcHours-(ptrHandlerRtc->RTC_config.rtcHours%10))/10;
//	 // convertimos el formato de Minutos en BDC
//	 MU = (ptrHandlerRtc->RTC_config.rtcMinutes%10);
//	 MD = (ptrHandlerRtc->RTC_config.rtcMinutes-(ptrHandlerRtc->RTC_config.rtcMinutes%10))/10;
//	 // convertimos el formato de segundos en BDC
//	 SU = (ptrHandlerRtc->RTC_config.rtcSeconds%10);
//	 SD = (ptrHandlerRtc->RTC_config.rtcSeconds-(ptrHandlerRtc->RTC_config.rtcSeconds%10))/10;
//	 // convertimos el formato de meses en BDC
//	 MoU = (ptrHandlerRtc->RTC_config.rtcMonth%10);
//	 MoD = (ptrHandlerRtc->RTC_config.rtcMonth-(ptrHandlerRtc->RTC_config.rtcMonth%10))/10;
//	 // convertimos el formato de dias en BDC
//	 DU = (ptrHandlerRtc->RTC_config.rtcDay%10);
//	 DD = (ptrHandlerRtc->RTC_config.rtcDay-(ptrHandlerRtc->RTC_config.rtcDay%10))/10;
//	 //Convertimos el formato de años en BBC
//	 YU = (ptrHandlerRtc->RTC_config.rtcYear%10);
//	 YD = (ptrHandlerRtc->RTC_config.rtcYear-(ptrHandlerRtc->RTC_config.rtcYear%10))/10;
//}

uint8_t actualTime[7];

uint8_t *getDate(void){
	uint8_t Hour  = 0;
	uint8_t Minutes = 0;
	uint8_t Seconds = 0;
	uint8_t Year = 0;
	uint8_t Month = 0;
	uint16_t Day = 0;
	uint8_t Weekday = 0;

	uint32_t Time = 0;
	uint32_t Date = 0;


	Time = RTC->TR;
	Date = RTC->DR;

	Hour	= (Time & 0x3F0000) >> RTC_TR_HU_Pos;
	Minutes = (Time & 0x007F00) >> RTC_TR_MNU_Pos;
	Seconds = Time  & 0x7F;
	Weekday = (Date & 0xE000)   >> RTC_DR_WDU_Pos;
	Year    = (Date & 0xFF0000) >> RTC_DR_YU_Pos;
	Month   = (Date & 0x1F00)   >> RTC_DR_MU_Pos;
	Day     =  Date  & 0x3F;

	actualTime[0] = Seconds;
	actualTime[1] = Minutes;
	actualTime[2] = Hour;
	actualTime[3] = Weekday;
	actualTime[4] = Day;
	actualTime[5] = Month;
	actualTime[6] = Year;

	return actualTime;
}
