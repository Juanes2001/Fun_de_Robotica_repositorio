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


// Estructura donde se colocan las distancias paralela y diagonal entre baldosas
typedef struct {
	float parallelDistance;
	float diagonalDiastance;
	uint8_t numberOfRows;
	uint8_t numberOfColumns;
	char **shorterWay;
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
	volatile int arbitraryPosition[2];
}costChangesAndPos_t;

//Matriz de arrays donde se almacenan respectivamente los valores de G cost, el F cost, y el H cost, posicion i del parent y posicion j del parent
extern float ***costs;
// Matriz donde se copiará la matriz que se inserta desde la terminal.
extern char **readableGrid;


// Funciones a Usar

// en el algoritmo es necesario si se da el caso cambiar el orden de prioridad al calcular el G cost actual y el F cost,
// Por lo que el pariente puede cambiar dependiendo si esto se cumple o no.
void updateParent(costChangesAndPos_t *ptrChanges, int posIJ[2]);
//Esta funcion renueva el costo G del punto desde el punto de analisis,
void updateGcost(AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges, int posIJ[2]);
// Esta funcion renueva el costo F del punto desde el punto de analisis.
void updateFcost( costChangesAndPos_t *ptrChanges, int posIJ[2]);
// Esta función Busca el punto inicial de la cuadricula entregada.
int findStart(char **Grid, AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges);
// Esta funcion busca el punto final o punto de llegada.
int findEnd(char **Grid, AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges);
// Esta funcion define la heuristica de cada cuadro de la cuadricula exceptuando los obstaculos
int setHeuristic(AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges);
// Con esta funcion calculamos el Gcost a partir del punto de analisis , el G cost es la distancia mas corta que hay pasando siempre por el
//punto de analisis y llendo hasta el punto de inicio
float setGcost (AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges, int posIJ[2]);
// Con esta funcion seteamos el F cost de los puntos aledanios al punto de analisis , incluyendo el punto de analisis
float setFcost (costChangesAndPos_t *ptrChanges, int posIJ[2]);
// Con etsa funcion seteamos el parent de los aledanios , es decir, la posicion a la que apuntan los aledanios a estudiar a partir del punto de analisis
void setParents (costChangesAndPos_t *ptrChanges, int posIJ[2]);
//Con esta funcion repartimos la cantidad de memoria que necesita el arreglo de char shorterWay
char** buildArrayShorterWay(int numElements, char **shorterWayArray);
// Esta función define la matriz que se usara para definir el camino mas corto, traida desde comunicacion serial.
char **buildMatrixCopy(AStar_distancesHandler *parameters, char **Grid);
// Con esta funcion allocamos la supermatriz donde se almacenaran los datos numericos de la heuristica el G cost y el F cost, posicion i y j del parent
float ***buildMatrixCosts(AStar_distancesHandler *parameters, float ***matrixCosts);
//Esta funcion me cuenta la cantidad de filas que tiene mi arreglo entrado M
uint8_t getRows(char **terminalGrid);
// Esta funciom retorna la cantidad de columnas que hay en el arreglo entrado N
uint8_t getColums(char **terminalGrid);
// Esta funcion halla la posicion del valor mas pequeño más pequeño dentro de un conjunto de valores
void findLesserValue(char*typeCost ,costChangesAndPos_t*ptrChanges, float ***matrixCosts);
//Esta funcion inicializa los parametros de la estructura AStar_distances
char* findShorterWay(char** terminalGrid, AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges);





#endif /* ASTAR_H_ */
