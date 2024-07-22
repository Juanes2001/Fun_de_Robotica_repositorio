/*
 * MPUAccel.c
 *
 *  Created on: 26/11/2022
 *      Author: ALEJANDRA MARIA
 */

#include "MPUAccel.h"
#include "I2CDriver.h"
#include "GPIOxDriver.h"


void configMPUAccel (MPUAccel_Config *ptrMPUAccel){

	uint8_t rdy = 0;

	// Paso 1 se configuran los pines GPIO y el perisferico I2C respectivo para poder configurar a partir del
	//MCU el sensor, se tiene en cuenta tanto la lectura del pin SDA como el SCL

	GPIO_Config(ptrMPUAccel->ptrGPIOhandler);
	i2c_config(ptrMPUAccel->ptrI2Chandler);

	// verificamos que el MPU se comunica con exito
	while(!rdy){

		rdy = WHOIAM(ptrMPUAccel);
	}


	// Paso 2 Inicializamos correctamente el acelerometro usando la comunicacion con este y sus registros,
	// Reseteamos el MPU en el registro 0x6B Reseteando el bit 7, es decir 0b1 << 7 negado
	writeData(ptrMPUAccel, 0x6B , ~(0b1 << 7));

	//Paso 3 , configuramos el Full Rage de la cantidad pedida en el registro 0x1B y 0x1C para GYRO y Accel
	// respectivamente

	switch (ptrMPUAccel->cantidad){

		case ACELERACION :{

			switch (ptrMPUAccel->fullScale){

				case ACCEL_2G :{

					writeData(ptrMPUAccel, 0x1C, ACCEL_2G);

					break;
				}
				case ACCEL_4G :{

					writeData(ptrMPUAccel, 0x1C, ACCEL_4G);

					break;
				}
				case ACCEL_8G :{

					writeData(ptrMPUAccel, 0x1C, ACCEL_8G);

					break;
				}
				case ACCEL_16G :{

					writeData(ptrMPUAccel, 0x1C, ACCEL_2G);

					break;
				}

				default:{

					break;
				}

			}

			break;
		}
		case GIROSCOPIO :{



			break;
		}
		case TEMPERATURA :{



			break;
		}

		default:{

			break;
		}

	}



	//Paso 4, Preguntamos que cantidad se desea medir y se activa dentro del registro


}



// En esta funcion escribimos la secuencia para escritura propia del MCU
void writeData (MPUAccel_Config *ptrMPUAccel, uint8_t RA, uint8_t data){
	//Limpiamos la bandera AF por si antes se levanto
	ptrMPUAccel->ptrI2Chandler->ptrI2Cx->SR1 &= ~(I2C_SR1_AF);

	//Comenzamos la transacción
	i2c_startTransaction (ptrMPUAccel->ptrI2Chandler);

	//Mandamos el Address correspondiente y el bit escribir
	i2c_sendSlaveAddressRW(ptrMPUAccel->ptrI2Chandler, ADDRESS_DOWN, I2C_WRITE_DATA);

	// Dentro de la funcion anterior ya esta la espera respectiva para el ACK que debe mandar el Slave
	i2c_sendMemoryAddress(ptrMPUAccel->ptrI2Chandler, RA);

	// Esperamos el ACK que debe de mandar el Slave
	while (ptrMPUAccel->ptrI2Chandler->ptrI2Cx->SR1 & I2C_SR1_AF){
		__NOP();
	}

	i2c_sendDataByte(ptrMPUAccel->ptrI2Chandler, data);

	// Esperamos el ACK que debe de mandar el Slave
	while (ptrMPUAccel->ptrI2Chandler->ptrI2Cx->SR1 & I2C_SR1_AF){
		__NOP();
	}

	i2c_stopTransaction(ptrMPUAccel->ptrI2Chandler);



}

// En esta funcion escribimos la secuencia para lectura propia del MCU
uint8_t readData (MPUAccel_Config *ptrMPUAccel, uint8_t RA){

	/* 0. Creamos una variable auxiliar para recribir el dato que leemos*/
	uint8_t auxRead = 0;

	//Limpiamos la bandera AF por si antes se levanto
	ptrMPUAccel->ptrI2Chandler->ptrI2Cx->SR1 &= ~(I2C_SR1_AF);

	//Comenzamos la transacción
	i2c_startTransaction (ptrMPUAccel->ptrI2Chandler);

	//Mandamos el Address correspondiente y el bit escribir
	i2c_sendSlaveAddressRW(ptrMPUAccel->ptrI2Chandler, ADDRESS_DOWN, I2C_WRITE_DATA);

	// Dentro de la funcion anterior ya esta la espera respectiva para el ACK que debe mandar el Slave
	i2c_sendMemoryAddress(ptrMPUAccel->ptrI2Chandler, RA);

	// Esperamos el ACK que debe de mandar el Slave
	while (ptrMPUAccel->ptrI2Chandler->ptrI2Cx->SR1 & I2C_SR1_AF){
		__NOP();
	}

	// Comenzamos el reestar
	i2c_reStartTransaction(ptrMPUAccel->ptrI2Chandler);

	// Ya dentro de la anterior funcion esta la espera a que comience el bit de start
	//Volvemos a mandar el Address con el bit de read (1)
	i2c_sendSlaveAddressRW(ptrMPUAccel->ptrI2Chandler, ADDRESS_DOWN, I2C_READ_DATA);

	// Mandamos el noAcknowledge despues de recibir el dato respectivo
	i2c_sendNoAck(ptrMPUAccel->ptrI2Chandler);

	// Paramos la transacción
	i2c_stopTransaction(ptrMPUAccel->ptrI2Chandler);

	auxRead = i2c_readDataByte(ptrMPUAccel->ptrI2Chandler);

	return auxRead;
}

//Esta funcion permite verificar comunicacion correcta con el MPU
uint8_t WHOIAM (MPUAccel_Config *ptrMPUAccel){

	uint8_t whoami = 0;
	uint8_t rdy    = 0;

	//leemos el registro 0x75 correspondiente
	whoami = readData(ptrMPUAccel, 0x75);

	// verificamos la transacción

	whoami &= ~(129);

	uint8_t rdy = (ADDRESS_DOWN >> 1) && (whoami >> 1);

	return rdy;

}





