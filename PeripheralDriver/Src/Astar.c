/*
 * Astar.c
 *
 *  Created on: Aug 13, 2024
 *      Author: juan
 */

#include "Astar.h"

#include <stdio.h>
#include <stdlib.h>


char* findShorterWay(AStar_distancesHandler *parameters, char** Grid){

	// Primero seteamos dentro de los valores de los parametros cuales son los valores de las filas y las columnas
	parameters->numberOfRows    = getRows(Grid);
	parameters->numberOfColumns = getColums(Grid);

	//Segundo construimos nuestra matriz dinamicamente repartida
	char ** matrix = buildMatrix(parameters, Grid);

	//Tercero seteamos nuetsra matriz que almacenara los datos de Gcost F cost, los costos
	//Variables que dependen del analisis respectivo,y el H cost que es la heuristica el cual es un valor
	// fijo Se tendra entonces una matriz de arrays donde se almacenaran
	// los valores como siguen, [Gcost, Fcost, Hcost]


    // Allocate memory for the matrix (Array of pointers)
    costs = (float ***)malloc(parameters->numberOfRows * sizeof(float **));
    for (int i = 0; i < parameters->numberOfRows; i++) {
    	costs[i] = (float **)malloc(parameters->numberOfColumns * sizeof(float*));
    	for (int j = 0; j < parameters->numberOfColumns; i++) {
    	    	costs[i][j] = (float *)malloc(3 * sizeof(float));
		}
    }

}

// con esta funcion seteamos la matriz Heuristica con la cual usaremos la info para buscar la ruta mas corta
void setHeuristic(void *matrixPointFirtsPos, costChangesAndPos_t *ptrChages){

	// La dinamica sera la siguiente, para cada entrada de la heuristica nos centraremos en recorrer cada entrada y almacenar en la tercera po-
	//sicion de cada fila y columna el valor de la heuristica dependiendo de donde este el punto de termino o End point


}

// En esta funcion nos centraremos en buscar la posicion i,j donde se almacena el punto de inicio del robot
int* findStart(void *matrixPointFirtsPos, costChangesAndPos_t *ptrChages){



}

int* findEnd(void *matrixPointFirtsPos, costChangesAndPos_t *ptrChages){


}

//Con esta funcion se reparte la memoria para la matriz de entrada desde la terminal serial

char **buildMatrx(AStar_distancesHandler *parameters, char **string){

	// Matriz donde se almacenaran las filas de String donde esta la informacion de los espacios libres y los obstaculos
	char **infoGrid = (char ** ) malloc(parameters->numberOfRows * sizeof(char *));
	for (uint8_t i = 0 ; i < parameters->numberOfRows; i++){
		infoGrid [i] = (char *)malloc(parameters->numberOfColumns *sizeof(char));
	}

	// Seteamos los valores dentro de la matriz infoGrid de la entrada respectiva
	for (uint8_t i = 0; i < parameters->numberOfRows; i++){
		for(uint8_t j = 0; j < parameters->numberOfColumns; j++){

			infoGrid[i][j] = string[i][j];

		}
	}

	return infoGrid;

}



// Se define la funcion de tomar cantidad de filas recorriendo la cantidad de String que tenga el puntero de arreglos matrix hasta que se
// encuentre con el puntero nulo.
uint8_t getRows(char **matrix){

	uint8_t counterRows = 0;
	while(matrix[counterRows] != NULL){

		counterRows++;

	}

	return counterRows;
}

//Se define la funcion de tomar cantidad de columnas recorriendo el string hasta encontrar el elemento nulo char
uint8_t getColums(char **matrix){

	uint8_t counterColumns = 0;
	while(matrix[0][counterColumns] != '\0'){

		counterColumns++;

	}

	return counterColumns;
}


// Función Allocate memory



// Función liberacion de memoria de las matrices doble puntero
void freeMatrix(int **matrix, int rows) {
    for (int i = 0; i < rows; i++) {
        free(matrix[i]);
    }
    free(matrix);
}