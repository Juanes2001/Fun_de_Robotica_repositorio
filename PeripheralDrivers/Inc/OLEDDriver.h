/*
 * OLEDDriver.h
 *
 *  Created on: 6/11/2022
 *      Author: ALEJANDRA MARIA
 */

#ifndef OLEDDRIVER_H_
#define OLEDDRIVER_H_

#include <stm32f4xx.h>
#include "I2CDriver.h"

#define OLED_CONTROLBYTE_CONFIG  0x00
#define OLED_CONTROLBYTE_DISPLAY 0x40

#define OLED_PAGE_0  0X00
#define OLED_PAGE_1  0X01
#define OLED_PAGE_2  0X02
#define OLED_PAGE_3  0X03
#define OLED_PAGE_4  0X04
#define OLED_PAGE_5  0X05
#define OLED_PAGE_6  0X06
#define OLED_PAGE_7  0X07


#endif /* OLEDDRIVER_H_ */


void sendDataBytes(I2C_Handler_t *ptrHandlerI2C, char *dataToSent, uint8_t sizeArray);
void sendCommandByte ( I2C_Handler_t *ptrHandlerI2C, char command);

//Funciones para la pantalla OLED

void startOLED (I2C_Handler_t *ptrHandlerI2Ctr);
void stopOLED (I2C_Handler_t *ptrHandlerI2Ctr);
void setPage (I2C_Handler_t *ptrHandlerI2Ctr, uint8_t page);
void setColumnAddress (I2C_Handler_t *ptrHandlerI2Ctr, uint8_t page);
void clearDisplay (I2C_Handler_t *ptrHandlerI2Ctr);
void setLineAddress (I2C_Handler_t *ptrHandlerI2Ctr, uint8_t moveNum);
void drawMSG (I2C_Handler_t *ptrHandlerI2Ctr,char *msg);
void drawSinglePageMSG (I2C_Handler_t *ptrHandlerI2Ctr, char *msg,uint8_t page);
char *letterTochar (uint8_t character);



