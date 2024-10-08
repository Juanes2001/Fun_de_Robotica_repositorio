

/*
 * I2CDriver.c
 *
 *  Created on: 20/10/2022
 *      Author: Juan Esteban Rodriguez Ochoa
 *      Email: jrodriguezoc@unal
 */


#include <stdint.h>
#include <math.h>

#include "I2CDriver.h"
#include "DMA.h"


uint8_t flagRx = 0;
uint8_t flagTx = 0;
DMA_Handler_t *ptrDMA_handler[2];

/*
 * Recordad que se debe configurar los pines para el I2C (SDA Y SCL),
 * para lo cual se necesita el modulo GPIO y los pines configurados
 * en el modo ALternate Function.
 * Ademas, estos pines deben ser configurados como salidas open-drain
 * y con la resistencias en modo pull-up.
 */
void i2c_config(I2C_Handler_t *ptrHandlerI2C){

	// Definimos los punteros para el DMA, si es necesario
//
//	ptrDMA_handler[0]->ptrDMAType = DMA1;
//	ptrDMA_handler[0]->ptrDMAStream = DMA1_Stream0;
//
//	ptrDMA_handler[1]->ptrDMAType = DMA1;
//	ptrDMA_handler[1]->ptrDMAStream = DMA1_Stream6;

	/* 1 Activamos la señal de reloj para el modulo I2C seleccionado*/
	if(ptrHandlerI2C->ptrI2Cx == I2C1){
		RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN;
	}

	else if(ptrHandlerI2C->ptrI2Cx == I2C2){
		RCC -> APB1ENR  |= RCC_APB1ENR_I2C2EN;
	}

	else if(ptrHandlerI2C->ptrI2Cx == I2C3){
		RCC -> APB1ENR |= RCC_APB1ENR_I2C3EN;
	}



	/* 0. desactivamos el modulo I2C */
	ptrHandlerI2C->ptrI2Cx->CR1 &= ~I2C_CR1_PE;


	while(ptrHandlerI2C->ptrI2Cx->SR2 & I2C_SR2_BUSY){
		__NOP();
	}

	/* 2. Reiniciamos el periferico, de forma que inicia en un estado conocido */
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_SWRST;

	__NOP();

	ptrHandlerI2C->ptrI2Cx->CR1 &= ~ I2C_CR1_SWRST;

	/*3. Indicamos cual es la velocidad del reloj principal, que es la señal utilizada
	 * por el periferico para generar la señal de reloj para el bus I2C */


	ptrHandlerI2C->ptrI2Cx->CR2 &= ~(0b111111 << I2C_CR2_FREQ_Pos);	// Borramos la configuracion previa.

	//Preguntamos que velocidad de reloj de tiene actualmente
	if (ptrHandlerI2C->I2C_Config.clkSpeed ==  MAIN_CLOCK_16_MHz_FOR_I2C ){

		ptrHandlerI2C->ptrI2Cx->CR2 |= (MAIN_CLOCK_16_MHz_FOR_I2C << I2C_CR2_FREQ_Pos);

	}else if(ptrHandlerI2C->I2C_Config.clkSpeed == MAIN_CLOCK_25_MHz_FOR_I2C){

		ptrHandlerI2C->ptrI2Cx->CR2 |= (MAIN_CLOCK_25_MHz_FOR_I2C << I2C_CR2_FREQ_Pos);

	}else if (ptrHandlerI2C->I2C_Config.clkSpeed == MAIN_CLOCK_50_MHz_FOR_I2C){

		ptrHandlerI2C->ptrI2Cx->CR2 |= (MAIN_CLOCK_50_MHz_FOR_I2C << I2C_CR2_FREQ_Pos);

	}



	/* 4. Configuramos el modo I2C en el que el sistema funciona
	 * En esta configuracion se incluye tambien la velocidad del reloj
	 * y el tiempo máximo para el cambio de la señal (T-Rise).
	 * Todo comienza con los dos registros en 0
	 */

	ptrHandlerI2C->ptrI2Cx->CCR = 0;
	ptrHandlerI2C->ptrI2Cx->TRISE = 0;

	if(ptrHandlerI2C->I2C_Config.modeI2C == I2C_MODE_SM){

		//Estamos en modo "standar" (SM Mode)
		// Seleccionamos el modo estandar
		ptrHandlerI2C->ptrI2Cx->CCR &= ~ (I2C_CCR_FS);

		//configuramos el registro que se encarga de generar la señal del reloj
		switch(ptrHandlerI2C->I2C_Config.clkSpeed){

			case MAIN_CLOCK_4_MHz_FOR_I2C:{

				ptrHandlerI2C->ptrI2Cx->CCR |=(I2C_MODE_SM_SPEED_100KHz_4MHz << I2C_CCR_CCR_Pos);

				//Configuramos el registro que controla el tiempo T-Rise máximo
				ptrHandlerI2C->ptrI2Cx->TRISE |= I2C_MAX_RISE_TIME_SM_4MHz;

				break;
			}case MAIN_CLOCK_16_MHz_FOR_I2C:{

				ptrHandlerI2C->ptrI2Cx->CCR |=(I2C_MODE_SM_SPEED_100KHz_16MHz << I2C_CCR_CCR_Pos);

				//Configuramos el registro que controla el tiempo T-Rise máximo
				ptrHandlerI2C->ptrI2Cx->TRISE |= I2C_MAX_RISE_TIME_SM_16MHz;

				break;
			}case MAIN_CLOCK_25_MHz_FOR_I2C:{

				ptrHandlerI2C->ptrI2Cx->CCR |=(I2C_MODE_SM_SPEED_100KHz_25MHz << I2C_CCR_CCR_Pos);

				//Configuramos el registro que controla el tiempo T-Rise máximo
				ptrHandlerI2C->ptrI2Cx->TRISE |= I2C_MAX_RISE_TIME_SM_25MHz;

				break;
			}case MAIN_CLOCK_50_MHz_FOR_I2C:{

				ptrHandlerI2C->ptrI2Cx->CCR |=(I2C_MODE_SM_SPEED_100KHz_50MHz << I2C_CCR_CCR_Pos);

				//Configuramos el registro que controla el tiempo T-Rise máximo
				ptrHandlerI2C->ptrI2Cx->TRISE |= I2C_MAX_RISE_TIME_SM_50MHz;


				break;
			}default:{

				break;
			}

		}


	}
	else{


		//Estamos en modo "Fast" (FM Mode)
		//Seleccioanmo el modo Fast
		ptrHandlerI2C->ptrI2Cx->CCR |=  I2C_CCR_FS;

		//configuramos el registro que se encarga de generar la señal del reloj
		switch(ptrHandlerI2C->I2C_Config.clkSpeed){

			case MAIN_CLOCK_4_MHz_FOR_I2C:{

				ptrHandlerI2C->ptrI2Cx->CCR |=(I2C_MODE_FM_SPEED_400KHz_4MHz << I2C_CCR_CCR_Pos);

				//Configuramos el registro que controla el tiempo T-Rise máximo
				ptrHandlerI2C->ptrI2Cx->TRISE |= I2C_MAX_RISE_TIME_FM_4MHz;

				break;
			}case MAIN_CLOCK_16_MHz_FOR_I2C:{

				ptrHandlerI2C->ptrI2Cx->CCR |=(I2C_MODE_FM_SPEED_400KHz_16MHz << I2C_CCR_CCR_Pos);

				//Configuramos el registro que controla el tiempo T-Rise máximo
				ptrHandlerI2C->ptrI2Cx->TRISE |= I2C_MAX_RISE_TIME_FM_16MHz;

				break;
			}case MAIN_CLOCK_25_MHz_FOR_I2C:{

				ptrHandlerI2C->ptrI2Cx->CCR |=(I2C_MODE_FM_SPEED_400KHz_25MHz << I2C_CCR_CCR_Pos);

				//Configuramos el registro que controla el tiempo T-Rise máximo
				ptrHandlerI2C->ptrI2Cx->TRISE |= I2C_MAX_RISE_TIME_FM_25MHz;

				break;
			}case MAIN_CLOCK_50_MHz_FOR_I2C:{

				ptrHandlerI2C->ptrI2Cx->CCR |= (I2C_MODE_FM_SPEED_400KHz_50MHz << I2C_CCR_CCR_Pos);

				//Configuramos el registro que controla el tiempo T-Rise máximo
				ptrHandlerI2C->ptrI2Cx->TRISE |= I2C_MAX_RISE_TIME_FM_50MHz;


				break;
			}default:{

				break;
			}

		}

	}

	while(ptrHandlerI2C->ptrI2Cx->SR2 & I2C_SR2_BUSY){
		__NOP();
	}

	/* 5. Activamos el modulo I2C */
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_PE;


}


/* 8. Generamos la condicion de stop */
void i2c_stopTransaction(I2C_Handler_t *ptrHandlerI2C){
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_STOP;
}


void i2c_startTransaction(I2C_Handler_t *ptrHandlerI2C){

	// Activamos el DMA request segun lo pedido por el usuario
	if (ptrHandlerI2C->I2C_Config.dma_Request){
		// Si estamos aqui e sporque queremos usar la DMA para la transaccion efectiva de datos
		ptrHandlerI2C->ptrI2Cx->CR2 |= I2C_CR2_DMAEN;
	}else{
		// Si estamos aqui es porque no queremos usar un DMA request para la transaccion de datos
	}

	/* 1. Verificamos que la linea no esta ocupada - bit "busy" en I2C_SR2 */
	while(ptrHandlerI2C->ptrI2Cx->SR2 & I2C_SR2_BUSY){
		__NOP();
	}

	/* 2. Genereamos la señal "start" */
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_START;

	/* 2a. Esperamos a que la bandera del evento "start" se levante
	 * Mientras esperamos, el valor de SB es 0, entonces la negacion (!) es 1*/
	while(!(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_SB)){
		__NOP();
	}
}

/**/
void i2c_reStartTransaction(I2C_Handler_t *ptrHandlerI2C){

	// Activamos el DMA request segun lo pedido por el usuario
	if (ptrHandlerI2C->I2C_Config.dma_Request){
		// Si estamos aqui e sporque queremos usar la DMA para la transaccion efectiva de datos
		ptrHandlerI2C->ptrI2Cx->CR2 |= I2C_CR2_DMAEN;
	}else{
		// Si estamos aqui es porque no queremos usar un DMA request para la transaccion de datos
	}

	/*2. Generamos la señal "start" */
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_START;

	/* 2a. Esperamos a que la bandera del evento "start" se levante*/
	/* Mientras esperamos, el valor de SB es 0, entonces la negacion es 1 */
	while(!(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_SB)){
		__NOP();
	}
}

/* 7a. Activamos la indicacion para no-ACK (indicacion para el Slave de terminar) */
void i2c_sendNoAck(I2C_Handler_t *ptrHandlerI2C){
	/*(Debemos escribir cero en la posicion ACK del registro de control 1) */
	ptrHandlerI2C->ptrI2Cx->CR1 &= ~(I2C_CR1_ACK);
}

/* 7b. Activamos la indicacion para ACK (indicacion para el Slave de terminar)*/
void i2c_sendAck(I2C_Handler_t *ptrHandlerI2C){
	/* (Debemos escribir uno en la posicion ACK del registro de control 1)*/
	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_ACK;
}

/**/
void i2c_sendSlaveAddressRW(I2C_Handler_t *ptrHandlerI2C, uint8_t slaveAddress, uint8_t readOrWrite){
	/* 0. Definimos una variable auxiliar */
	uint8_t auxByte = 0;
	(void) auxByte;

	/* 3. Enviamos la direccion del Slave y el bit que indica que deseamos escribir (0) */
	/* (en el siguiente paso se envia la direccion de memoria que se desea escribir  */
	ptrHandlerI2C->ptrI2Cx->DR = (slaveAddress << 1) | readOrWrite;

	/* 3.1 Esperamos hasta que la bendera del evento "addr" se levante
	 * (esto nos indica que la direccion fue enviada satisfactoriamente
	 */
	while( !(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_ADDR)){
		__NOP();
	}

	/* 3.2 Debemos limpiar la bandera de la recepcion de ACK de la "addr", para lo cual
	 * debemos leer en secuencia el I2C_SR1 y luego I2C_SR2
	 */
	auxByte = ptrHandlerI2C->ptrI2Cx->SR1;
	auxByte = ptrHandlerI2C->ptrI2Cx->SR2;

}

/**/
void i2c_sendMemoryAddress(I2C_Handler_t *ptrHandlerI2C, uint8_t memAddr){

	/* 4. Enviamos la direccion de memoria qe deseamos leer */
	ptrHandlerI2C->ptrI2Cx->DR = memAddr;

	/* 4.1 Esoeramos hasta que el byte sea transmitido */
	while( !(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_TXE)){
		__NOP();
	}
}

/**/
void i2c_sendDataByte(I2C_Handler_t *ptrHandlerI2C, uint8_t dataToWrite){
	/* 5. Cargamos el valor que deseamos escribir */
	ptrHandlerI2C->ptrI2Cx->DR = dataToWrite;

	/* 6. Esperamos hasta que el byte sea transmitido */
	while( !(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_BTF)){
		__NOP();
	}
}

uint8_t i2c_readDataByte(I2C_Handler_t *ptrHandlerI2C){
	/*9. Esperamos hasta que el byte entrante sea recibido */
	while( !(ptrHandlerI2C->ptrI2Cx->SR1 & I2C_SR1_RXNE)){
		__NOP();
	}

	ptrHandlerI2C->I2C_Config.dataI2C = ptrHandlerI2C->ptrI2Cx->DR;
	return ptrHandlerI2C->I2C_Config.dataI2C;
}

/**/
uint8_t i2c_readSingleRegister(I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead){

	/* 0. Creamos una variable auxiliar para recribir el dato que leemos*/
	uint8_t auxRead = 0;

	/* 1. Generamos la condicion Start */
	i2c_startTransaction(ptrHandlerI2C);

	/* 2. Enviamos la direccion del eslavo y la direccion de ESCRIBIR */
	i2c_sendSlaveAddressRW(ptrHandlerI2C, ptrHandlerI2C->I2C_Config.slaveAddress, I2C_WRITE_DATA);

	/* 3. Enviamos la direccion de memoria que deseamos leer */
	i2c_sendMemoryAddress(ptrHandlerI2C, regToRead);

	/* 4. Creamos una condicion re Start */
	i2c_startTransaction(ptrHandlerI2C);

	/* 5. Enviamos la direccion del escalvo y la indicacion de LEER */
	i2c_sendSlaveAddressRW(ptrHandlerI2C, ptrHandlerI2C->I2C_Config.slaveAddress, I2C_READ_DATA);

	/* 6. Generamos la condicion de NoACK, para que el Master no responda y el slave solo envie 1 byte*/
	i2c_sendNoAck(ptrHandlerI2C);

	/* 7. Generamos la condicion Stop, para que el slave se detenga despues de 1 byte */
	i2c_stopTransaction(ptrHandlerI2C);

	/* 8. Leemos el dato que envia el escalvo */
	auxRead = i2c_readDataByte(ptrHandlerI2C);

	return auxRead;
}

/**/
void i2c_writeSingleRegister(I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead, uint8_t newValue){

	/* 1. Generamos la condicion Start */
	i2c_startTransaction(ptrHandlerI2C);

	/* 2. Enviamos la direccion del esclavo y la indicacion de ESCRIBIR */
	i2c_sendSlaveAddressRW(ptrHandlerI2C, ptrHandlerI2C->I2C_Config.slaveAddress, I2C_WRITE_DATA);

	/* 3. Enviamos la direccion de memoria que deseamos escribir */
	i2c_sendMemoryAddress(ptrHandlerI2C, regToRead);

	/* 4. Enviamos el valor que deseamos escribir en el registro seleccionado */
	i2c_sendDataByte (ptrHandlerI2C, newValue);

	/* 5. Generamos la condicion Stop, para que el slave se detenga despues de 1 byte */
	i2c_stopTransaction(ptrHandlerI2C);

}


void DMA1_Stream0_Callback_Rx(void){
	// Con este calback buscamos el uso simple de banderas para continuar con la transaccion de datos de forma segura
	flagRx = SET;
}

void DMA1_Stream6_Callback_Tx(void){
	// Con este calback buscamos el uso simple de banderas para continuar con la transaccion de datos de forma segura
	flagTx = SET;
}





