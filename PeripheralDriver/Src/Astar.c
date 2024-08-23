/*
 * Astar.c
 *
 *  Created on: Aug 13, 2024
 *      Author: juan
 */

#include "Astar.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"


char* findShorterWay(char** terminalGrid,char**Gridcopy, float*** matrixCosts , AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges){

	// seteamos las variables locales a usar
	char nineSlotsMatriz[3][3]; // matriz que tomara una parte de redeableGrid para analisis
	uint8_t shorterWayFound = RESET;
	uint8_t i = 0;
	uint8_t j = 0;
	int position[2];
	uint8_t numberOfPositions = 0;
	char character = '\0';
	uint8_t counter = 0;
	//matriz donde se almacenaran en orden ascendente los F cost de las posiciones en estado de Open, esta si tendra un valor maximo y dos columnas, donde
	// Se almacenara el F cost en la primera y el Hcost en la segunda,
	float decisionMatrix[100][4];


	// Primero seteamos dentro de los valores de los parametros cuales son los valores de las filas y las columnas
	parameters->numberOfRows    = getRows(terminalGrid);
	parameters->numberOfColumns = getColums(terminalGrid);

	//Segundo construimos nuestra matriz dinamicamente repartida
	buildMatrixCopy(parameters, terminalGrid, Gridcopy);

	//Tercero seteamos nuetsra matriz que almacenara los datos de Gcost F cost, los costos
	//Variables que dependen del analisis respectivo,y el H cost que es la heuristica el cual es un valor
	// fijo Se tendra entonces una matriz de arrays donde se almacenaran
	// los valores como siguen, [Gcost, Fcost, Hcost]

    // Allocate memory for the matrix (Array of pointers)
	buildMatrixCosts(parameters, matrixCosts);

    //Cuarto, seteamos la matriz heuristica, la cual es la ultima matriz de el bloque de tres de la matriz de costos
    if (setHeuristic(parameters, ptrChanges, matrixCosts, Gridcopy)){
    	// Si estamos aqui todo salio correctamente, el programa puede seguir su curso
    	__NOP();
    }else{
    	// Si estamos aqui es porque no se encontro el punto final en el redeableGrid.
    	return (char *) NULL;
    }

    // Seteada la heuristica AQUI COMIENZA EL ALGORITMO A TRABAJAR, seteamos el punto de inicio y lo guardamos dentro de la estructura
    // correspondiente
    if(findStart(Gridcopy, parameters, ptrChanges)){
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


    while(!shorterWayFound){

    	// este while actuara como un while de recorrido lineal, donde se busca analizar las posiciones i,j de los aledaños recorriendo cada punto linealmente
    	// al punto de analisis

    	//Actualizamos el punto de analisis para seguir analizando a partir del punto de análisis siguiente
    	ptrChanges->posAnalisis[0] = ptrChanges->posOpen[0];
    	ptrChanges->posAnalisis[1] = ptrChanges->posOpen[1];

    	// guardamos la matriz 3x3 de redeableGrid para analisis, para posiciones no correctas se colocan tales posiciones en 0 tipo char
    	// Y para posiciones ya analizadas se coloca una D de done.
    	for(i = 0; i < 3; i++){
    		for (j = 0; j < 3; j++){

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
    			}else if(i == 1 && j == 1){
    				// Este caso corresponde con el punto de analisis , este caso no lo queremos estudiar ya que ya estaria estudiado como tal,
    				// queremos es solo estudiar sus aledanios, por lo que en esta posición colocaremos una 'P' como de Point
    				nineSlotsMatriz[i][j] = 'P';
    			}else{
    				// Este ultimo caso donde no se cumple lo anterior  simplemente copiamos exactamente lo mismo que aparece en la matriz de caracteres
    				nineSlotsMatriz[i][j] = Gridcopy[ptrChanges->posAnalisis[0] + i - 1][ptrChanges->posAnalisis[1] + j - 1];
    			}
    		}// Termino del ciclo for
    	}// Termino del ciclo for

    	// Estando aqui ya tendremos la matriz 3x3 correctamente constituida donde se tiene la informacion tanto del punto de analisis
    	// (posicion central) como de sus aledanios, de ella se puede aplicar correctamente el algoritmo.

    	// Lo que sigue sera volver a recorrer tal matriz repitiendo las condiciones , pero esta vez leyendo directamente el contenido de
    	// Esta matriz, Se estudiaran los siguientes casos:

    	for(i = 0; i < 3; i++){
			for (j = 0; j < 3; j++){
				switch (nineSlotsMatriz[i][j]) {
					case '#':{
						// El primer caso seria cuando el puntero Que estudia a la matriz 3x3 corresponde con un obstaculo, tal pisicion simpkemente se
						//ignora
						// No se hace nada ya que una posicion se obstaculo no se analiza
						__NOP();
						break;
					}case 'D':{
						// El segundo caso seria cuando el puntero que estudia la matriz corresponde con un punto ya cerrado o ya pasado a la lista de
						// posiciones ya analizadas, por lo que tampoco se hace nada con este caso,
						__NOP();
						break;
					}case '0':{
						// El tercer caso seria cuando el puntero que estudia la matriz corresponde con un punto 0 de tipo char que indica que
						// es una posicion por fuera del rango, por lo que tampoco se hace nada con este caso,
						__NOP();
						break;
					}case 's':{
						// El cuarto caso seria cuando el puntero señala la posicion de start por lo que tampoco se hace nada con este caso,
						__NOP();
						break;
					}case 'P':{
						// El quinto caso seria cuando el puntero señala la posicion de analisis, dentro del analisis no queremos
						// que este se analice a si mismo, por lo que ignoramos este punto, solo nos intrresan sus aledanios
						__NOP();
						break;
					}case 'O':{
						// El Sexto caso nos encontramos con un estado Open , por lo que tenemos que recalcular para el punto de
						// analisis el Gcost y el Fcost
						position[0] = i;
						position[1] = j;
						// Como la posicion estudiada esta en estado de open , se debe volver a calcular el G cost y el fcost correspondiente
						// a la posicion opened con respecto al punto de analisis
						ptrChanges->Fcost = setFcost(ptrChanges, position, matrixCosts);
						// Estudiamos si el nuevo FCost es mayor menor o igual al Fcost que ya tiene el estado abierto
						if (ptrChanges->Fcost >= matrixCosts[ptrChanges->posAnalisis[0] + i - 1][ptrChanges->posAnalisis[0] + j - 1][1]){
							// Si el Fcost es mayor o igual a el anteriormente calculado, Significa que el Gcost calculado es el mismo o mayor,
							//por lo que no se actualiza ni el G cost y el F cost ni el parent
							__NOP();
						}else{
							// Si estamos aqui es porque el Fcost calculado para el G cost nuevo es mejor y se debe actualizar tanto el Gcost el Fcost
							// como el parent.
							updateGcost(parameters, ptrChanges, position, matrixCosts);
							updateFcost(ptrChanges, position, matrixCosts);
							updateParent(ptrChanges, position, matrixCosts);

							// Si si actualizamos la posicion abierta respectiva, tambien se debe actualizar en la matriz de decisión el F cost
							decisionMatrix[matrixCosts[ptrChanges->posAnalisis[0] + i - 1][ptrChanges->posAnalisis[1] + j - 1][5]][0] =
									       matrixCosts[ptrChanges->posAnalisis[0] + i - 1][ptrChanges->posAnalisis[1] + j - 1][1]; // La segunda matriz son los F costs

						}
						break;
					}case '*':{
						// El séptimo caso seria cuando el puntero que estudia la matriz corresponde con un punto '*' que corresponde con un
						// espacio no estudiado, por lo que simplemente se setea sobre estos nuevos puntos su Gcost y su Fcost, incluyendo el parent
						updateGcost(parameters, ptrChanges, position, matrixCosts);
						updateFcost(ptrChanges, position, matrixCosts);
						updateParent(ptrChanges, position, matrixCosts);

						// Es conveniente que guardemos tambien esta información dentro de readableGrid porque se necesita luego almacenar esta ifnromacion ya
						// estudiada
						Gridcopy[ptrChanges->posAnalisis[0] + i - 1][ptrChanges->posAnalisis[1] + j - 1] = 'O';

						// Como se crearon nuevos estados abiertos , se almacena dentro de la matriz de decicion el Fcost, el H cost, y la posicion
						// de cada punto analizado
						// se almacena el Fcost
						decisionMatrix[counter][0] = matrixCosts[ptrChanges->posAnalisis[0] + i - 1][ptrChanges->posAnalisis[1] + j - 1][1];
						// Se almacena el H cost
						decisionMatrix[counter][1] = matrixCosts[ptrChanges->posAnalisis[0] + i - 1][ptrChanges->posAnalisis[1] + j - 1][2];
						// Se almacena la posicion i
						decisionMatrix[counter][2] = ptrChanges->posAnalisis[0] + i - 1;
						// Se almacena la posicion j
						decisionMatrix[counter][3] = ptrChanges->posAnalisis[1] + j - 1;

						// Guardamos en la matris grande de costos en la ultima matriz el valor del counter asignado a la posición
						matrixCosts[ptrChanges->posAnalisis[0] + i - 1][ptrChanges->posAnalisis[1] + j - 1][5] = counter;

						// Aumentamos el contador ya que cada posicion estudiada aqui es una nueva posicion que entra a estado open
						counter++;

						break;
					}case 'e':{
						// El octavo caso Sería cuando se encuentra la 'e' de end , donde ya logramos despues de viajar a traves de la maya llegar
						// hasta el punto final, aqui para economizar recursos simplemente haremos que el ciclo se detenga y de parent al end le asignamos
						// el ultimo
						position[0] = i;
						position[1] = j;
						updateParent(ptrChanges, position, matrixCosts);
						shorterWayFound = SET;
						break;
					}
					default:{
						// Si se llega hasta aca es porque hay un caracter no permitido dentro de la malla y se debe parar la ejecucion del programa
						return (char*) NULL;
						break;
					}
				}// Termino del switch case

				if (shorterWayFound){
					break;
				}

			}// Termino del ciclo for
			if (shorterWayFound){
				break;
			}
		}// Termino del ciclo for

    	// Estando aqui ya se han analizado todos los aledanios del punto de analisis, por lo que podemos seleccionar de los aledanios quien es
    	// el que tiene el Fcost mas pequeño, y en caso de Fcost iguales desempata el que tenga una heuristica o Hcost menor
    	// Volvemos a recorrer los aledanios para seleccionar la nueva posicion, tal posicion pasara a ser el nuevo punto de analisis, el resto
    	// seran solo puntos en estado Open, y el antiguo punto de analisis sera ahora un punto Done, Para ello usaremos la funcion findeLesserValue
    	// Esta funcion almacena dentro de la estructura costChangesAndPos_t los valores de la pisicion del F cost mas pequeño, en el caso de que haya
    	// mas de uno igual, se alzará una bandera que indica que esto ocurrio, por lo que internamente el tambien analiza cual es la heuristica
    	if (!shorterWayFound){
			findLesserValue(ptrChanges, matrixCosts, decisionMatrix, counter + 1);

			// A partir de aqui ya tendriamos la posicion del F cost mas pequeño, pero primero se compueba de que si depronto hubo un F cost igual
			if (ptrChanges->equalFcost){
				// Si estamos aqui es porque si hubo mas de un Fcost igual , por lo que se busca es la posicion del Hcost mas pequeño como la nueva posicion
				// que tomara el nuevo papel de punto de analisis, y el que fue el punto de analisis sera ahor aun estado Done o 'D'
				// Llevamos la posicion de analisis al estado cerrado
				ptrChanges->posClosed[0] = ptrChanges->posAnalisis[0];
				ptrChanges->posClosed[1] = ptrChanges->posAnalisis[1];

				// Convertimos el estado estudiado en un estado Done acualizando el redeableGrid, excepto cuando se trata del punto de start
				if (Gridcopy[ptrChanges->posAnalisis[0]][ptrChanges->posAnalisis[1]] == 's'){
					// Dejamos el char de start tal cual como esta
					__NOP();
				}else{
					// Si no se trata del caracter de Start si actualizamos con el caracter de Done 'D'
					Gridcopy[ptrChanges->posAnalisis[0]][ptrChanges->posAnalisis[1]] = 'D';
				}

				//Actualizamos el punto de analisis con la posicion de la heuristica mas pequeña
				ptrChanges->posAnalisis[0] = ptrChanges->lesserHcostPosition[0];
				ptrChanges->posAnalisis[1] = ptrChanges->lesserHcostPosition[1];

				//Colocamos en estado de open el nuevo estado a estudiar
				ptrChanges->posOpen[0] = ptrChanges->posAnalisis[0];
				ptrChanges->posOpen[1] = ptrChanges->posAnalisis[1];

			}else{
				// Si el programa no entra en este if quiere decir uqe ya se encontro la ruta mas corta y ya es hora de construir la matriz de posiciones
				// donde se almacenara la ruta mas corta y se devuelve como return
				//buscamos cuantos elementos deberia de tener el arreglo para ello usaremos el siguiente while donde recorreremos desde el end hasta el
				//start
				i = ptrChanges->endPos[0];
				j = ptrChanges->endPos[1];
				while(Gridcopy[i][j] != 's'){
					// Actualizamos a la nueva posición
					i = matrixCosts[i][j][3];
					j = matrixCosts[i][j][4];
					// incrementamos en uno la cantidad de posiciones a guardar
					numberOfPositions++;
				}
			}
    	}

    }// final del ciclo While

    // estando aqui ya solo queda almacenar toda las posiciones parent comenzando desde el end hasta el start, siguendo el parent de cada uno se asegura
    // que lo que se esta almacenando es la ruta mas corta


    // Seteamos primero la memoria de el arreglo de posiciones
    buildArrayShorterWay(numElements, shorterWayArray);








}

// esta funcion actuazliza en la matriz de costs el parent correspondiente
void updateParent(costChangesAndPos_t *ptrChanges, int posIJ[2],float*** matrixCosts){
	setParents(ptrChanges, posIJ);

	matrixCosts[ptrChanges->posAnalisis[0] + posIJ[0] - 1][ptrChanges->posAnalisis[1] + posIJ[1] - 1][3] = ptrChanges->parent[0]; //Posicion i del parent
	matrixCosts[ptrChanges->posAnalisis[0] + posIJ[0] - 1][ptrChanges->posAnalisis[1] + posIJ[1] - 1][4] = ptrChanges->parent[1]; //Posicion j del parent

}

// esta funcion actualiz el Gcost correspondiente
void updateGcost(AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges, int posIJ[2],float*** matrixCosts ){
	//Por ultimo se setea en la pisicion de la heuristica correspondiente a la matriz ultima de la super matriz
	// de costos
	matrixCosts[ptrChanges->posAnalisis[0] + posIJ[0] -1][ptrChanges->posAnalisis[1] + posIJ[1] -1][0] = setGcost(parameters, ptrChanges, posIJ);
}

// Esta función actualiza el Fcost correspondiente
void updateFcost(costChangesAndPos_t *ptrChanges, int posIJ[2],float*** matrixCosts ){
	//Por ultimo se setea en la pisicion de la heuristica correspondiente a la matriz ultima de la super matriz
	// de costos
	matrixCosts[ptrChanges->posAnalisis[0] + posIJ[0] -1][ptrChanges->posAnalisis[1] + posIJ[1] -1][1] = setFcost(ptrChanges, posIJ, matrixCosts);

}

// con esta funcion seteamos la matriz Heuristica con la cual usaremos la info para buscar la ruta mas corta
int setHeuristic(AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges,float*** matrixCosts , char** Gridcopy){

	// definimos variables locales
	int distRows     = 0;
	int distColumns  = 0;
	int distanceToGo = 0;

	// La dinamica sera la siguiente, para cada entrada de la heuristica nos centraremos en recorrer cada entrada y almacenar en la tercera po-
	//sicion de cada fila y columna el valor de la heuristica dependiendo de donde este el punto de termino o End point

	// Comenzamos entonces recorriendo cada posicion del terccer bloque matricial de ***costs
	// recorremos las filas
	// Almacenamos la posicion final
	if(findEnd(Gridcopy, parameters, ptrChanges)){
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
				matrixCosts[i][j][2] = distanceToGo;
			}else{
				// Si estamos aqui es porque la distancia del robot a la columna es mas cercana que del mismo a la fila, por lo que
				// lo unico que cambia es que las veces que hay que ir diagonal sera hasta tocar la columna del end
				distanceToGo = parameters->diagonalDiastance * distColumns;
				// Lo que falta para llegar al end es el recorrido por toda la columna, es decir el restante entre
				// distRows y distColumns
				distanceToGo += parameters->parallelDistance * abs(distRows-distColumns);
				matrixCosts[i][j][2] = distanceToGo;
			}

		}

	}

	// Terminado TODO el recorrido se puede salir de la funcion y decir que todo fue correcto
	return SET;

}

//Con esta funcion se halla el Gcost teniendo en cuenta la posicion de analisis
float setGcost (AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges, int posIJ[2]){
	// definimos variables locales
	int distRows     = 0;
	int distColumns  = 0;
	float distanceToGo = 0;

	// Luego calculamos el Gcost partiendo de que se tiene que pasar siempre por la posicion de analisis
	// Se analiza cual es la distancia que hay entre el punto de analisis y el punto de start
	distRows    = abs(ptrChanges->posAnalisis[0] -ptrChanges->startPos[0]);
	distColumns = abs(ptrChanges->posAnalisis[1]-ptrChanges->startPos[1]);

	if ((ptrChanges->posAnalisis[0] + posIJ[0] -1) != ptrChanges->posAnalisis[0]
	 || (ptrChanges->posAnalisis[1] + posIJ[1] -1) != ptrChanges->posAnalisis[1]){
		// Si estamos aqui es porque estamos en una de las 4 esquinas aledanias, por lo que la distancia a la columna o la fila
		// mas cercana a el punto de analisis es 1, se tendra que ir diagonalmente
		distanceToGo = parameters->diagonalDiastance;

		// Luego se calcula la distancia que resta aplicando el mismo algoritmo de la heuristica
		//pero esta vez para el punto de analisis hasta el punto de inicio

		if (distRows <= distColumns){
			// Si estamos aca es porque el robot en la posicion i,j en la que estaria esta mas cerca de la fila del end
			// que de la columna del end, por lo que se debe ir diagonal
			distanceToGo += parameters->diagonalDiastance * distRows;
			// Luego el robot ya se encontraria en la misma fila del end, por lo que faltaria sumarle las posiciones restantes
			// paralelas hasta llegar a la columna , la distancia que falta seria la resta de las diferencias en valor absoluto
			// En el caso de que las distancias sean iguales, el robot solo ira diagonal hasta el end , no necesitara ir paralelo.
			distanceToGo += parameters->parallelDistance * abs(distRows-distColumns);
		}else{
			// Si estamos aqui es porque la distancia del robot a la columna es mas cercana que del mismo a la fila, por lo que
			// lo unico que cambia es que las veces que hay que ir diagonal sera hasta tocar la columna del end
			distanceToGo += parameters->diagonalDiastance * distColumns;
			// Lo que falta para llegar al end es el recorrido por toda la columna, es decir el restante entre
			// distRows y distColumns
			distanceToGo += parameters->parallelDistance * abs(distRows-distColumns);
		}

	}else {
		// Si estamos aqui es porque estamos en una de las cuatro aristas, donde tanto el puntero como la posicion real del punto de analisis
		// coinciden en fila o en columna, la unica diferencia aqui es que se calcula paralelamente al punto de analisis y a partir de ahi
		// se aplica el algoritmo de la heuristica
		distanceToGo = parameters->parallelDistance;

		if (distRows <= distColumns){
			distanceToGo += parameters->diagonalDiastance * distRows;
			distanceToGo += parameters->parallelDistance * abs(distRows-distColumns);
		}else{
			distanceToGo += parameters->diagonalDiastance * distColumns;
			distanceToGo += parameters->parallelDistance * abs(distRows-distColumns);
		}
	}
	ptrChanges->Gcost = distanceToGo;

	return ptrChanges->Gcost;
}

// Con esta funcion seteamos el F cost en la matriz 2 de la posicion correspondiente
float setFcost (costChangesAndPos_t *ptrChanges, int posIJ[2], float*** matrixCosts){

	// Esta funcion es simple ya que solo tenemos que calcular de la matriz 3x3 de analisis y sumar el H cost y el G cost para tener el F cost
	ptrChanges->Fcost = matrixCosts[ptrChanges->posAnalisis[0]+ posIJ[0] -1][ptrChanges->posAnalisis[1]+ posIJ[1] -1][0]  // Gcost
			          + matrixCosts[ptrChanges->posAnalisis[0]+ posIJ[0] -1][ptrChanges->posAnalisis[1]+ posIJ[1] -1][2]; // Hcost

	return ptrChanges->Fcost;

}

// Con esta funcion seteamos la posicion del parent de los aledanios,
void setParents (costChangesAndPos_t *ptrChanges, int posIJ[2]){
	// esta funcion es simple, ya que solo tenemos que setear de la matriz 3x3 de analisis y colocar en la posicion 4 y 5 el indice i y j correspondiente
	// al parent de cada punto aledanio, exceptuando el punto de analisis ya que ese tiene un parent propio

	if ((ptrChanges->posAnalisis[0] + posIJ[0] - 1) == ptrChanges->posAnalisis[0]
	&&  (ptrChanges->posAnalisis[1] + posIJ[1] - 1) == ptrChanges->posAnalisis[1] ){
		// Aqui no hacemos nada porque significa que estamos sobre el punto de analisis, y no queremos cambiarle el parent a este
		__NOP();
	}else{
		// Si estamos aqui es porque estamos en alguno de los puntos aledanios
		ptrChanges->parent[0] = ptrChanges->posAnalisis[0];
		ptrChanges->parent[1] = ptrChanges->posAnalisis[1];
	}

}

// En esta funcion nos centraremos en buscar la posicion i,j donde se almacena el punto de inicio del robot
int findStart(char **Gridcopy, AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges){

	// Buscamos dentro de la matriz Grid que corresponde con la copia que le hacemos a la matriz de strings de entrada
	// en la terminal
	// recorremos la matriz hasta encontrar la posicion correpondiente con un char 's' de start
	for (uint8_t i = 0; i<parameters->numberOfRows; i++){
		for(uint8_t j = 0; j< parameters->numberOfColumns; j++){
			if (Gridcopy[i][j] == 's'){
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
int findEnd(char **Gridcopy, AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges){


	// Buscamos dentro de la matriz Grid que corresponde con la copia que le hacemos a la matriz de strings de entrada
	// en la terminal
	// recorremos la matriz hasta encontrar la posicion correpondiente con un char 'e' de end
	for (uint8_t i = 0; i<parameters->numberOfRows; i++){
		for(uint8_t j = 0; j< parameters->numberOfColumns; j++){
			if (Gridcopy[i][j] == 'e'){
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

// Con esta funcion se reparte la memoria para el arreglo de entrada, para almacenar asi la ruta mas corta
void buildArrayShorterWay(int numElements, char **shorterWayArray){

	//Repartimos la memoria teniendo en cuenta el numero de elementos que corresponden a la ruta mas corta seguida, cada entrada
	// almacenará las posiciones que debe de seguir el robot incluyendo el punto de inicio y final
	shorterWayArray = (char ** ) malloc(numElements * sizeof(char *));
	for (uint8_t i = 0 ; i < numElements; i++){
		shorterWayArray [i] = (char *)malloc(2 *sizeof(char));
	}

}

//Con esta funcion se reparte la memoria para la matriz de entrada desde la terminal serial

void buildMatrixCopy(AStar_distancesHandler *parameters, char **terminalGrid, char** Gridcopy){

	// Matriz donde se almacenaran las filas de String donde esta la informacion de los espacios libres y los obstaculos
	Gridcopy = (char ** ) malloc(parameters->numberOfRows * sizeof(char *));
	for (uint8_t i = 0 ; i < parameters->numberOfRows; i++){
		Gridcopy [i] = (char *)malloc(parameters->numberOfColumns + 1 *sizeof(char));
	}

	// Seteamos los valores dentro de la matriz infoGrid de la entrada respectiva
	for (uint8_t i = 0; i < parameters->numberOfRows; i++){
		for(uint8_t j = 0; j < parameters->numberOfColumns + 1; j++){

			if (j == parameters->numberOfColumns){
				// Agregamos al final la terminacion nula para que cada fila sea un string completo
				Gridcopy[i][j] = '\0';
			}else{
				Gridcopy[i][j] = terminalGrid[i][j];
			}
		}
	}

}

//Con esta funcion repartimos la memoria para la matriz de costos
void buildMatrixCosts(AStar_distancesHandler *parameters, float ***matrixCosts){

	matrixCosts = (float***) malloc(parameters->numberOfRows * sizeof(float**));
	for (int i = 0; i < parameters->numberOfRows; i++) {
		matrixCosts[i] = (float **)malloc(parameters->numberOfColumns * sizeof(float*));
		for (int j = 0; j < parameters->numberOfColumns; i++) {
			matrixCosts[i][j] = (float *)malloc(6 * sizeof(float));
		}
	}

}



// Se define la funcion de tomar cantidad de filas recorriendo la cantidad de String que tenga el puntero de arreglos matrix hasta que se
// encuentre con el puntero nulo.
uint8_t getRows(char **terminalGrid){

	uint8_t counterRows = 0;
	while(terminalGrid[counterRows] != NULL){

		counterRows++;

	}

	return counterRows;
}

//Se define la funcion de tomar cantidad de columnas recorriendo el string hasta encontrar el elemento nulo char
uint8_t getColums(char **terminalGrid){

	uint8_t counterColumns = 0;
	while(terminalGrid[0][counterColumns] != '\0'){

		counterColumns++;

	}

	return counterColumns;
}

// esta funcion nos almacena en uno de los arrays volatiles de la estructura costChangesAndPos_t la posicion del valor Fcost o H cost mas pequeño,
// Se debe identificar con un string si se quiere hallar el Fcost mas pequeño o el Hcost mas pequeño, asi, "Fcost" si se quiere hallar el F cost o
// "Hcost" si se quiere hallar el H cost
void findLesserValue(costChangesAndPos_t *ptrChanges, float ***matrixCosts, float decisionMtrx[100][4], uint8_t counter){
	// seteamos las variables locales
	uint8_t counterRow = 0;
	uint8_t counterColumn = 0;
	uint8_t contadorDM = 0;
	uint8_t i;
	uint8_t j;
	uint8_t falseAlarm = RESET;
	uint8_t IfoundIt = RESET;
	float value = 0;

	// counter es la entrada que nos representa la cantidad de posiciones regristradas en la matriz de decision hasta ahora


	// El algoritmo que se usará es que se recorrerá cada una de las posiciones y se analizara con las demas , excpliyendo obviamente
	// la posicion central
	while (!IfoundIt){



		while(contadorDM < counter){

			for(i = 0; i<3 ; i++){
				for(j = 0; j<3 ; j++){
					if ((ptrChanges->posAnalisis[0] + counterRow - 1) == (ptrChanges->posAnalisis[0] + i - 1)
					&&  (ptrChanges->posAnalisis[1] + counterColumn - 1) == (ptrChanges->posAnalisis[1] + j - 1)){
						// Si estamos aqui es porque estamos analizando el punto de analisis, por lo que lo ignoramos
						__NOP();
					}else{
						// Si estamos aca es porque podemos hacer la comparación
						if (matrixCosts[ptrChanges->posAnalisis[0] + counterRow - 1][ptrChanges->posAnalisis[1] + counterColumn - 1][1]
						  < matrixCosts[ptrChanges->posAnalisis[0] + i - 1][ptrChanges->posAnalisis[1] + j - 1][1]){
							// Si estamos aca es porque efectivamente el Fcost es menor o igual al resto de Fcost
						}else if (matrixCosts[ptrChanges->posAnalisis[0] + counterRow - 1][ptrChanges->posAnalisis[1] + counterColumn - 1][1]
							   == matrixCosts[ptrChanges->posAnalisis[0] + i - 1][ptrChanges->posAnalisis[1] + j - 1][1]){
							// Si estamos aqui es porque el programa encontro mas minimo un valor igual al analizado
							ptrChanges->equalFcost = SET;
						}else{
							// Si estamos aqui es porque el programa hallo almenos un valor menor al analizado, por lo que no nos sirve
							// Se resetea la bandera que decia que habia un vakor igual, esto debe de ser solo cierto si el valor que es
							// igual es el menor de la matriz
							ptrChanges->equalFcost = RESET;
							falseAlarm = SET;
							break;
						}
					}

				}//Terminacion de un for
				// Este condicional es solo para acabar el ciclo for de las filas y se continue a que se compare la siguiente posición
				if (falseAlarm){
					falseAlarm = RESET;
					break;
				}

			}//Terminacion del otro for

			if (i == 3 && j == 3){
				// Si entramos aqui es porque se logro analizar toda la matriz y se encontro la posicion que corresponde con la posicion mas pequeña
				// de todas, tambien guardamos el valor mas pequeño, si este se repite, se sabra gracias a la bandera
				ptrChanges->lesserFcostPosition[0] = counterRow;
				ptrChanges->lesserFcostPosition[1] = counterColumn;
				ptrChanges->lesserFcost = matrixCosts[ptrChanges->posAnalisis[0] + counterRow - 1][ptrChanges->posAnalisis[1] + counterColumn - 1][1];
				IfoundIt = SET;
			}else{
				if (counterColumn == 2){
					counterRow++;
					counterColumn = 0;
				}else{
					counterColumn++;
				}
			}

		}//Terminación del ciclo while

	}//Terminacion del ciclo while

	// Ahora hallaremos el valor mas pequeño de H cost, solo en el caso de que la bandera correspondiente se haya levantado
	if (ptrChanges->equalFcost){
		// Si estamos aqui es porque si hay mas de un valor de F cost que corresponde con el valor mas pequeño, desempatamos buscando el Hcost mas pequeño
		// Para ello recorreremos la matriz a analizar de nuevo pero esta vez solo buscando aquellos valores que correspondan con el valor hallado de Fcost
		counterRow = 0;
		counterColumn = 0;
		falseAlarm = RESET;
		IfoundIt = RESET;
		// reseteamos las variables locales y volvemos a empezar pero esta vez solo buscando aquellos casos donde se tenga el valor encontrado
		while (!IfoundIt){

			for(i = 0; i<3 ; i++){
				for(j = 0; j<3 ; j++){
					value = costs[ptrChanges->posAnalisis[0] + counterRow - 1][ptrChanges->posAnalisis[1] + counterColumn - 1][1] / ptrChanges->lesserFcost;
					if ((ptrChanges->posAnalisis[0] + counterRow - 1) == (ptrChanges->posAnalisis[0] + i - 1)
					&&  (ptrChanges->posAnalisis[1] + counterColumn - 1) == (ptrChanges->posAnalisis[1] + j - 1)){
						// Si estamos aqui es porque estamos analizando el punto de analisis, por lo que lo ignoramos
						__NOP();
					}else if (value == 1){
						// Si estamos aca es porque podemos hacer la comparación pero esta vez con la matriz heuristica
						if (matrixCosts[ptrChanges->posAnalisis[0] + counterRow - 1][ptrChanges->posAnalisis[1] + counterColumn - 1][2]
						  <= matrixCosts[ptrChanges->posAnalisis[0] + i - 1][ptrChanges->posAnalisis[1] + j - 1][2]){
							// Si estamos aca es porque efectivamente el Hcost es menor o igual al resto de Hcost
							__NOP();
						}else{
							// Si estamos aqui es porque el programa hallo almenos un valor menor al analizado, por lo que no nos sirve
							falseAlarm = SET;
							break;
						}
					}
					else{
						// Estamos en un valor que no nos interesa evaluar, por lo que lo ignoramos y salimos
						falseAlarm = SET;
						break;
					}

				}//Terminacion de un for

				// Este condicional es solo para acabar el ciclo for de las filas y se continue a que se compare la siguiente posición
				if (falseAlarm){
					falseAlarm = RESET;
					break;
				}

			}//Terminacion del otro for

			if (i == 3 && j == 3){
				// Si entramos aqui es porque se logro analizar toda la matriz y se encontro la posicion que corresponde con la posicion mas pequeña
				// de todas, tambien guardamos el valor mas pequeño, si este se repite, se sabra gracias a la bandera
				ptrChanges->lesserHcostPosition[0] = counterRow;
				ptrChanges->lesserHcostPosition[1] = counterColumn;
				ptrChanges->lesserHcost = matrixCosts[ptrChanges->posAnalisis[0] + counterRow - 1][ptrChanges->posAnalisis[1] + counterColumn - 1][2];
				IfoundIt = SET;
			}else{
				if (counterColumn == 2){
					counterRow++;
					counterColumn = 0;
				}else{
					counterColumn++;
				}
			}

		}//Terminacion del ciclo while
	}else{
		// Si estamos aca es porque no se alzo la bandera que indica que hay mas de un F cost igual por lo que no hacemos nada
		__NOP();
	}


}


