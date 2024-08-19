/*
 * Astar.c
 *
 *  Created on: Aug 13, 2024
 *      Author: juan
 */

#include "Astar.h"

#include <stdio.h>
#include <stdlib.h>
#include "math.h"


char* findShorterWay(char** Grid, AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges){

	// seteamos las variables locales a usar
	int counterRow   = -1;
	int conterColumn = -1;
	char nineSlotsMatriz[3][3]; // matriz que tomara una parte de redeableGrid para analisis
	uint8_t shorterWayFound = RESET;

	// Primero seteamos dentro de los valores de los parametros cuales son los valores de las filas y las columnas
	parameters->numberOfRows    = getRows(Grid);
	parameters->numberOfColumns = getColums(Grid);

	//Segundo construimos nuestra matriz dinamicamente repartida
	readableGrid = buildMatrixCopy(parameters, Grid);

	//Tercero seteamos nuetsra matriz que almacenara los datos de Gcost F cost, los costos
	//Variables que dependen del analisis respectivo,y el H cost que es la heuristica el cual es un valor
	// fijo Se tendra entonces una matriz de arrays donde se almacenaran
	// los valores como siguen, [Gcost, Fcost, Hcost]

    // Allocate memory for the matrix (Array of pointers)
    costs = buildMatrixCosts(parameters, costs);

    //Cuarto, seteamos la matriz heuristica, la cual es la ultima matriz de el bloque de tres de la matriz de costos
    if (setHeuristic(parameters, ptrChanges)){
    	// Si estamos aqui todo salio correctamente, el programa puede seguir su curso
    	__NOP();
    }else{
    	// Si estamos aqui es porque no se encontro el punto final en el redeableGrid.
    	return (char *) NULL;
    }

    // Seteada la heuristica AQUI COMIENZA EL ALGORITMO A TRABAJAR, seteamos el punto de inicio y lo guardamos dentro de la estructura
    // correspondiente
    if(findStart(readableGrid, parameters, ptrChanges)){
    	// Si estamos aqui es porque se encontro el punto de inicio con exito
    	__NOP();
    }else{

    	// Si estamos aqui no hay punto de inicio y no se puede realizar el algoritmo
    	return (char *) NULL;
    }

    // Comenzamos analizando los vecinos del punto de inicio, calculando para cada uno de ellos (incluyendo el punto de inicio) el Gcosto
    // y el Fcost, de todos ellos al final se selecciona aquel que tenga el F cost mas pequeño, del siguiente while no se sale hasta que se tenga el
    // Array de patents correspondiente a la ruta mas corta

    //guardamos en la posicion de analisis la posicion de start
    ptrChanges->posAnalisis[0] = ptrChanges->startPos[0];
    ptrChanges->posAnalisis[1] = ptrChanges->startPos[1];
    // Definimos tambien el estado de abierto
    ptrChanges->posOpen[0] =  ptrChanges->startPos[0];
    ptrChanges->posOpen[1] =  ptrChanges->startPos[1];


    while(shorterWayFound){

    	// este while actuara como un while de recorrido circular, donde se busca analizar las posiciones i,j de los aledaños dando un circulo
    	// al punto de analisis

    	//Actualizamos el punto de analisis para seguir analisando a partir del punto de analisis siguiente
    	ptrChanges->posAnalisis[0] = ptrChanges->posOpen[0];
    	ptrChanges->posAnalisis[1] = ptrChanges->posOpen[1];

    	// guardamos la matriz 3x3 de redeableGrid para analisis, para posiciones no correctas se colocan tales posiciones en 0 tipo char
    	// Y para posiciones ya analizadas se coloca una D de done.
    	for(uint8_t i = 0; i < 3; i++){
    		for (uint8_t j = 0; j < 3; j++){

    			if 	  ((ptrChanges->posAnalisis[0] + i - 1) < 0
    			    || (ptrChanges->posAnalisis[1] + j - 1) < 0) {
    				// El primer caso seria cuando el puntero que estudia la matriz esta por fuera del rango , especificamente por detras del rango
    				// donde el indice seria negativo
    				nineSlotsMatriz[i][j] = '0';

    			}else if ((ptrChanges->posAnalisis[0] + i - 1) > parameters->numberOfRows - 1
    				||    (ptrChanges->posAnalisis[1] + j - 1) > parameters->numberOfColumns - 1){
    				// El segundo caso seria cuando el puntero que estudia la matriz esta por fuera del rango, especificamente por delante
    				// del rango donde el indice seria mayor al limite superior del numero de filas o columnas restado uno.
    				nineSlotsMatriz[i][j] = '0';
    			}else if ((ptrChanges->posAnalisis[0] + i - 1) == ptrChanges->posClosed[0]
					||    (ptrChanges->posAnalisis[1] + j - 1) == ptrChanges->posClosed[1]){
    				// Este tercer caso estudia si el puntero que estudia la matriz esta ya cerrado, por lo que la posicion correspondiente
    				// se manejara como una 'D' tipo char, para ya saber que esa posicion ha sido estudiada y cerrada
    				nineSlotsMatriz[i][j] = 'D';
    			}else if ((ptrChanges->posAnalisis[0] + i - 1) == ptrChanges->posClosed[0]
					||    (ptrChanges->posAnalisis[1] + j - 1) == ptrChanges->posClosed[1]){
    				// En este cuarto caso se estudia si el puntero que estudia la matri esta ya abierto, por lo que la posicion correspondiente
    				// Se manejara como una 'O'
    				nineSlotsMatriz[i][j] = 'O';
    			}else{
    				// Este ultimo caso donde no se cumple lo anterior  simplemente copiamos exactamente lo mismo que aparece en la matriz de caracteres
    				nineSlotsMatriz[i][j] = readableGrid[ptrChanges->posAnalisis[0] + i - 1][ptrChanges->posAnalisis[1] + j - 1];
    			}
    		}
    	}

    	// Estando aqui ya tendremos la matriz 3x3 correctamente constituida donde se tiene la informacion tanto del punto de analisis
    	// (posicion central) como de sus aledanios, de ella se puede aplicar correctamente el algoritmo.

    	// Lo que sigue

    	// Primero nos aseguramos que sus aledaños no sean obstaculos, por lo que son posiciones que no nos interesan
    	if (){


    	}


    }






}

// con esta funcion seteamos la matriz Heuristica con la cual usaremos la info para buscar la ruta mas corta
int setHeuristic(AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges){

	// definimos variables locales
	int distRows     = 0;
	int distColumns  = 0;
	int distanceToGo = 0;

	// La dinamica sera la siguiente, para cada entrada de la heuristica nos centraremos en recorrer cada entrada y almacenar en la tercera po-
	//sicion de cada fila y columna el valor de la heuristica dependiendo de donde este el punto de termino o End point

	// Comenzamos entonces recorriendo cada posicion del terccer bloque matricial de ***costs
	// recorremos las filas
	// Almacenamos la posicion final
	if(findEnd(readableGrid, parameters, ptrChanges)){
		// Se encontro la posicion final, y continua con el programa
		__NOP();
	}else{
		// no se encontro la posicion final, la funcion entonces no puede setear la heuristica y se sale
		return RESET;
	}

	// A partir de aqui ya se tiene la posicion del punto final y ya se puede entonces calcular la heuristica
	// partiendo siempre de que el robot puede ir en diagonal y luego en linea recta , siempre buscando la ruta mas eficiente

	for (uint8_t i = 0; i< parameters->numberOfRows; i++){
		for(uint8_t j = 0; j < parameters->numberOfColumns; j++){
			// para cada posicion i,j, se mira i-iend y j-jend, y se evalua cual de los dos es menor y luego
			// De los dos que sea mas pequeño, el robot tendra que ir diagonal hasta encontrarse con la fila o la columna
			// maas cercana a la fila o la columna del punto final, y ya luego sumarle la distancia paralela que falta para llegar al punto end
			distRows    = abs(i-ptrChanges->endPos[0]);
			distColumns = abs(j-ptrChanges->endPos[1]);
			if (distRows <= distColumns){
				// Si estamos aca es porque el robot en la posicion i,j en la que estaria esta mas cerca de la fila del end
				// que de la columna del end, por lo que se debe ir diagonal
				distanceToGo = parameters->diagonalDiastance * distRows;
				// Luego el robot ya se encontraria en la misma fila del end, por lo que faltaria sumarle las posiciones restantes
				// paralelas hasta llegar a la columna , la distancia que falta seria la resta de las diferencias en valor absoluto
				// En el caso de que las distancias sean iguales, el robot solo ira diagonal hasta el end , no necesitara ir paralelo.
				distanceToGo += parameters->parallelDistance * abs(distRows-distColumns);
				//Por ultimo se setea en la pisicion de la heuristica correspondiente a la matriz ultima de la sumer matriz
				// de costos
				costs[i][j][2] = distanceToGo;
			}else{
				// Si estamos aqui es porque la distancia del robot a la columna es mas cercana que del mismo a la fila, por lo que
				// lo unico que cambia es que las veces que hay que ir diagonal sera hasta tocar la columna del end
				distanceToGo = parameters->diagonalDiastance * distColumns;
				// Lo que falta para llegar al end es el recorrido por toda la columna, es decir el restante entre
				// distRows y distColumns
				distanceToGo += parameters->parallelDistance * abs(distRows-distColumns);
				costs[i][j][2] = distanceToGo;
			}

		}

	}

	// Terminado todo el recorrido se puede salir de la funcion y decir que todo fue correcto
	return SET;

}

// En esta funcion nos centraremos en buscar la posicion i,j donde se almacena el punto de inicio del robot
int findStart(char **Grid, AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges){

	// Buscamos dentro de la matriz Grid que corresponde con la copia que le hacemos a la matriz de strings de entrada
	// en la terminal
	// recorremos la matriz hasta encontrar la posicion correpondiente con un char 's' de start
	for (uint8_t i = 0; i<parameters->numberOfRows; i++){
		for(uint8_t j = 0; j< parameters->numberOfColumns; j++){
			if (readableGrid[i][j] == 's'){
				ptrChanges->startPos[0] = i;
				ptrChanges->startPos[1] = j;
				return SET;
			}else{
				__NOP();
			}

		}

	}
	// Si la funcion llego hasta aca es porque no encontro dentro del arreglo ninguna letra e
	return RESET;



}
int findEnd(char **Grid, AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges){


	// Buscamos dentro de la matriz Grid que corresponde con la copia que le hacemos a la matriz de strings de entrada
	// en la terminal
	// recorremos la matriz hasta encontrar la posicion correpondiente con un char 'e' de end
	for (uint8_t i = 0; i<parameters->numberOfRows; i++){
		for(uint8_t j = 0; j< parameters->numberOfColumns; j++){
			if (readableGrid[i][j] == 'e'){
				ptrChanges->endPos[0] = i;
				ptrChanges->endPos[1] = j;
				return SET;
			}else{
				__NOP();
			}

		}

	}
	// Si la funcion llego hasta aca es porque no encontro dentro del arreglo ninguna letra e
	return RESET;

}

//Con esta funcion se reparte la memoria para la matriz de entrada desde la terminal serial

char **buildMatrixCopy(AStar_distancesHandler *parameters, char **string){

	// Matriz donde se almacenaran las filas de String donde esta la informacion de los espacios libres y los obstaculos
	char **infoGrid = (char ** ) malloc(parameters->numberOfRows * sizeof(char *));
	for (uint8_t i = 0 ; i < parameters->numberOfRows; i++){
		infoGrid [i] = (char *)malloc(parameters->numberOfColumns + 1 *sizeof(char));
	}

	// Seteamos los valores dentro de la matriz infoGrid de la entrada respectiva
	for (uint8_t i = 0; i < parameters->numberOfRows; i++){
		for(uint8_t j = 0; j < parameters->numberOfColumns + 1; j++){

			if (j == parameters->numberOfColumns){
				// Agregamos al final la terminacion nula para que cada fila sea un string completo
				infoGrid[i][j] = '\0';
			}else{
				infoGrid[i][j] = string[i][j];
			}
		}
	}

	return infoGrid;

}

//Con esta funcion repartimos la memoria para la matriz de costos
float ***buildMatrixCosts(AStar_distancesHandler *parameters, float ***matrixCosts){

	float ***costMatrix = (float***) malloc(parameters->numberOfRows * sizeof(float**));
	for (int i = 0; i < parameters->numberOfRows; i++) {
		costMatrix[i] = (float **)malloc(parameters->numberOfColumns * sizeof(float*));
		for (int j = 0; j < parameters->numberOfColumns; i++) {
			costMatrix[i][j] = (float *)malloc(3 * sizeof(float));
		}
	}

	return costMatrix;
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


