/*
 * Astar.h
 *
 *  Created on: Aug 13, 2024
 *      Author: juan
 */

#ifndef ASTAR_H_
#define ASTAR_H_

// Includes
#include "stm32f4xx.h"
#include "USARTxDriver.h"
#include "GPIOxDriver.h"
#include "PosRobt.h"


// Estructura donde se colocan las distancias paralela y diagonal entre baldosas
typedef struct {
	// parametros definibles
	float parallelDistance;
	float diagonalDiastance;

	//Parametros a definir despues de entrar la malla
	uint8_t numberOfRows;
	uint8_t numberOfColumns;
	uint8_t numberOfElements;
}AStar_distancesHandler;


// Estructura donde se almacenan los costos variables de cada celda quitando la celda de los obstaculos
typedef struct{
	volatile float Gcost;  // Distancia que hay entre un punto y el punto de partida pasando por el punto de analisis
	volatile float Fcost;  // El costo de la heuristica con el G cost
	volatile int posAnalisis[2]; // Posicion elemento i, j de la matriz de posiciones, en estado de analisis,
	volatile int posOpen[2]; // Posicion elemento i, j de la matriz de posiciones, en estado de abierto,
	volatile int posClosed[2]; // Posicion elemento i, j de la matriz de posiciones, en estado de cerrado,
	volatile char parent[2]; // Este sera la posicion i,j que se le asignara a cada una de las posiciones estudiadas a partir del punto de analisis
	int startPos[2];// Posicion i,j del comienzo
	int endPos[2];//Posicion i,j del final
	volatile int lesserFcostPosition; // En este arreglo podemos subir la posicion del Fcost mas pequeño que queramos para usarla despues
	volatile int lesserHcostPosition; // En este arreglo podemos subir la posicion del Hcost mas pequeño que queramos para usarla despues
	uint8_t equalFcost;// Esta es una bandera que se usara si existen F cost iguales dentro de un arreglo, esto nos ayudara a desempatar
	float lesserFcost; // Con este valor podemos almacenar el valor del Fcost mas pequeño
	float lesserHcost; // Con este valor podemos almacenar el valor del Hcost mas pequeño
}costChangesAndPos_t;

//Matriz de arrays donde se almacenan respectivamente los valores de G cost, el F cost, y el H cost, posicion i del parent y posicion j del parent,
// Y al final la posicion counter que corresponde con la posicion de la fila de la posicion guardada
extern float costs[7][7][6];
// Matriz donde se copiará la matriz que se inserta desde la terminal.
extern char readableGrid[7][7];
// Matriz donde se almacenara las posicones de la ruta mas corta
extern int shorterWay[20][2];

extern USART_Handler_t handlerAstarUsart;
extern GPIO_Handler_t handlerAstarPinRx;
extern GPIO_Handler_t handlerAstarPinTx;




// Funciones a Usar

// en el algoritmo es necesario si se da el caso cambiar el orden de prioridad al calcular el G cost actual y el F cost,
// Por lo que el pariente puede cambiar dependiendo si esto se cumple o no.
void updateParent(costChangesAndPos_t *ptrChanges, int posIJ[2],float matrixCosts[7][7][6] );
//Esta funcion renueva el costo G del punto desde el punto de analisis,
void updateGcost(AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges, int posIJ[2],float matrixCosts[7][7][6] );
// Esta funcion renueva el costo F del punto desde el punto de analisis.
void updateFcost(AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges, int posIJ[2], float matrixCosts[7][7][6] );
// Esta función Busca el punto inicial de la cuadricula entregada.
int findStart(char Gridcopy[7][7], AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges);
// Esta funcion busca el punto final o punto de llegada.
int findEnd(char Gridcopy[7][7], AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges);
// Esta funcion define la heuristica de cada cuadro de la cuadricula exceptuando los obstaculos
int setHeuristic(AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges, float matrixCosts[7][7][6], char Gridcopy[7][7]);
// Con esta funcion calculamos el Gcost a partir del punto de analisis , el G cost es la distancia mas corta que hay pasando siempre por el
//punto de analisis y llendo hasta el punto de inicio
float setGcost (AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges, int posIJ[2]);
// Con esta funcion seteamos el F cost de los puntos aledanios al punto de analisis , incluyendo el punto de analisis
float setFcost (AStar_distancesHandler *parameters ,costChangesAndPos_t *ptrChanges, int posIJ[2],float matrixCosts[7][7][6]);
// Con etsa funcion seteamos el parent de los aledanios , es decir, la posicion a la que apuntan los aledanios a estudiar a partir del punto de analisis
void setParents (costChangesAndPos_t *ptrChanges, int posIJ[2]);
//Con esta funcion repartimos la cantidad de memoria que necesita el arreglo de char shorterWay
void buildArrayShorterWay(AStar_distancesHandler *parameters, int shorterWay[20][2]);
// Esta función define la matriz que se usara para definir el camino mas corto, traida desde comunicacion serial.
void buildMatrixCopy(AStar_distancesHandler *parameters, char terminalGrid[7][7], char Gridcopy[7][7]);
// Con esta funcion allocamos la supermatriz donde se almacenaran los datos numericos de la heuristica el G cost y el F cost, posicion i y j del parent
void buildMatrixCosts(AStar_distancesHandler *parameters, float matrixCosts[7][7][6]);
//Esta funcion me cuenta la cantidad de filas que tiene mi arreglo entrado M
uint8_t getRows(char terminalGrid[7][7]);
// Esta funciom retorna la cantidad de columnas que hay en el arreglo entrado N
uint8_t getColums(char terminalGrid[7][7]);
// Esta funcion halla la posicion del valor mas pequeño más pequeño dentro de un conjunto de valores
void findLesserValue(costChangesAndPos_t*ptrChanges, float decisionMtrx[30][4], uint8_t contador);
//Esta funcion inicializa los parametros de la estructura AStar_distances
int findShorterWay(char terminalGrid[7][7], char Gridcopy[7][7], float matrixCosts[7][7][6],
		AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges, int shorterWay[20][2]);
// Con esta funcion liberamos la memoria utilizada por la super matriz de costos
void freeCosts(AStar_distancesHandler *parameters, float***matrixCosts);
// Con esta funcion liberamos la memoria utilizada por la matriz que almacena la copia de los strings ingresados
void freeReadableGrid(AStar_distancesHandler *parameters, char** Gridcopy);
// con esta funcion liberamos la memoria utilizada por la matriz que almacena la ruta mas corta
void freeShorterWay(AStar_distancesHandler *parameters, int **shorterWayArray);
// Con las siguientes funciones inicializamos a los handler necesarios para poder usar la comunicacion serial desde aqui y no desde el main
void initSerialComunication (USART_Handler_t *ptrHandlerUsart, GPIO_Handler_t *ptrHandlerRx, GPIO_Handler_t *ptrHandlerTx);


///////////////////////////////////////////Funciones para realizar las operaciones///////////////////////////////////////
void create_Astar_operations(AStar_distancesHandler *parameters,
		          	  	  	 int shorterWayArray[20][2],
							 Parameters_Operation_t prtList[30],
							 Parameter_build_t *ptrbuild,
							 Parameters_Path_t *ptrPath,
							 Parameters_Position_t *ptrPos);




#endif /* ASTAR_H_ */
