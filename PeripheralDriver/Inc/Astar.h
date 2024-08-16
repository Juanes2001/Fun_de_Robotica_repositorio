/*
 * Astar.h
 *
 *  Created on: Aug 13, 2024
 *      Author: juan
 */

#ifndef ASTAR_H_
#define ASTAR_H_

// Includes
#include <stdio.h>
#include "stm32f4xx.h"


// Estructura donde se colocan las distancias paralela y diagonal entre baldosas
typedef struct {
	float parallelDistance;
	float diagonalDiastance;
	uint8_t numberOfRows;
	uint8_t numberOfColumns;
}AStar_distancesHandler;


// Estructura donde se almacenan los costos variables de cada celda quitando la celda de los obstaculos
typedef struct{
	float Gcost;  // Distancia que hay entre un punto y el punto de partida pasando por el punto de analisis
	float Fcost;  // El costo de la heuristica con el G cost
	int pos[1][2]; // Posicion elemento i, j de la matriz de posiciones,
}costChangesAndPos_t;

//Matriz de arrays donde se almacenan respectivamente los valores de G cost, el F cost, y el H cost
extern float ***costs;
// Matriz donde se copiará la matriz que se inserta desde la terminal.
extern char **readableGrid;


// Funciones a Usar

// en el algoritmo es necesario si se da el caso cambiar el orden de prioridad al calcular el G cost actual y el F cost,
// Por lo que el pariente puede cambiar dependiendo si esto se cumple o no.
void updateParent(void *matrixPointFirtsPos);
//Esta funcion renueva el costo G del punto desde el punto de analisis,
void updateGcost(void *matrixPointFirtsPos, costChangesAndPos_t *ptrChages);
// Esta funcion renueva el costo F del punto desde el punto de analisis.
void uptdateFcost(void *matrixPointFirtsPos, costChangesAndPos_t *ptrChages);
// Esta función Busca el punto inicial de la cuadricula entregada.
void findStart(void *matrixPointFirtsPos, costChangesAndPos_t *ptrChages);
// Esta funcion busca el punto final o punto de llegada.
void findEnd(void *matrixPointFirtsPos, costChangesAndPos_t *ptrChages);
// Esta funcion define la heuristica de cada cuadro de la cuadricula exceptuando los obstaculos
void setHeuristic(void *matrixPointFirtsPos, costChangesAndPos_t *ptrChages);
// Esta función define la matriz que se usara para definir el camino mas corto, traida desde comunicacion serial.
char **buildMatrixCopy(AStar_distancesHandler *parameters, char *string[]);
// Con esta funcion allocamos la supermatriz donde se almacenaran los datos numericos de la heuristica el G cost y el F cost
float ***buildMatrixCosts(AStar_distancesHandler *parameters, float ***matrixCosts);
//Esta funcion me cuenta la cantidad de filas que tiene mi arreglo entrado M
uint8_t getRows(char **matrix);
// Esta funciom retorna la cantidad de columnas que hay en el arreglo entrado N
uint8_t getColums(char **matrix);
//Esta funcion inicializa los parametros de la estructura AStar_distances
char* findShorterWay(AStar_distancesHandler *parameters, char** Grid);





#endif /* ASTAR_H_ */
