/*
 * Astar.c
 *
 *  Created on: Aug 13, 2024
 *      Author: juan
 */

#include "Astar.h"
#include <stdio.h>


void init(float parallel, float diagonal, AStar_distancesHandler *distances){

	//Primero seteamos nuetsra matriz que almacenara los datos de Gcost F cost, los costos
	//Variables que dependen del analisis respectivo,y el H cost que es la heuristica el cual es un valor
	// fijo Se tendra entonces una matriz de arrays donde se almacenaran
	// los valores como siguen, [Gcost, Fcost, Hcost]


    // Allocate memory for the matrix (Array of pointers)
    costs = (int **)malloc(M * sizeof(int **));
    for (int i = 0; i < M; i++) {
        matrix[i] = (int **)malloc(N * sizeof(int *));
        for (int j = 0; j < N; j++) {
            matrix[i][j] = (int *)malloc(K * sizeof(int));
        }
    }

    // Initialize and assign values to the matrix
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < K; k++) {
                matrix[i][j][k] = i * N * K + j * K + k; // Example assignment
            }
        }
    }

    // Access and print the matrix of arrays
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {
            printf("matrix[%d][%d] = { ", i, j);
            for (int k = 0; k < K; k++) {
                printf("%d ", matrix[i][j][k]);
            }
            printf("}\n");
        }
    }

    // Free the allocated memory
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {
            free(matrix[i][j]);
        }
        free(matrix[i]);
    }
    free(matrix);




}



// Se define la funcion de tomar cantidad de filas recorriendo la cantidad de String que tenga el puntero de arreglos infoGrid
uint8_t getRows(){

	uint8_t numRows;
	numRows = sizeof(infoGrid)/sizeof(infoGrid[0]);

	return numRows;
}

uint8_t getColums(){

	uint8_t numColums;
	numColums = sizeof(infoGrid[0])/sizeof(infoGrid[0][0]);

	return numColums;
}
