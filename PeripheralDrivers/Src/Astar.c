/*
 * Astar.c
 *
 *  Created on: Aug 13, 2024
 *      Author: juan
 */

#include "Astar.h"
#include "USARTxDriver.h"
#include "GPIOxDriver.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"

USART_Handler_t handlerAstarUsart = {0};

GPIO_Handler_t handlerAstarPinRx = {0};
GPIO_Handler_t handlerAstarPinTx = {0};

char buffer[64] = {0};

float costs[7][7][6]   = {0};
char readableGrid[7][7] = {0};
int shorterWay[20][2]     = {0};



int findShorterWay(char terminalGrid[7][7],
				   char Gridcopy[7][7],
				   float matrixCosts[7][7][6],
				   AStar_distancesHandler *parameters,
				   costChangesAndPos_t *ptrChanges,
				   int shorterWay[20][2]){

	writeMsg(&handlerAstarUsart, "\n______________Comencemos el viaje_____________\n");

	// seteamos las variables locales a usar
	char nineSlotsMatriz[3][3] = {0}; // matriz que tomara una parte de redeableGrid para analisis
	uint8_t shorterWayFound = RESET;
	uint8_t i = 0;
	uint8_t j = 0;
	int position[2];
	uint8_t numberOfPositions = 0;
	uint8_t counter = 0;
	uint8_t counterStudy = 0;
	//matriz donde se almacenaran en orden ascendente los F cost de las posiciones en estado de Open, esta si tendra un valor maximo y dos columnas, donde
	// Se almacenara el F cost en la primera y el Hcost en la segunda,
	float decisionMatrix[30][4] = {0};



	// Primero seteamos dentro de los valores de los parametros cuales son los valores de las filas y las columnas
	parameters->numberOfRows    = getRows(terminalGrid);
	parameters->numberOfColumns = getColums(terminalGrid);

	//Segundo construimos nuestra matriz dinamicamente repartida
	buildMatrixCopy(parameters, terminalGrid, Gridcopy);


	//Tercero seteamos nuetsra matriz que almacenara los datos de Gcost F cost, los costos
	//Variables que dependen del analisis respectivo,y el H cost que es la heuristica el cual es un valor
	// fijo Se tendra entonces una matriz de arrays donde se almacenaran
	// los valores como siguen, [Gcost, Fcost, Hcost]

    //Cuarto, seteamos la matriz heuristica, la cual es la ultima matriz de el bloque de tres de la matriz de costos
    if (setHeuristic(parameters, ptrChanges, matrixCosts, Gridcopy)){
    	// Si estamos aqui todo salio correctamente, el programa puede seguir su curso
    	__NOP();
    }else{
    	// Si estamos aqui es porque no se encontro el punto final en el redeableGrid.
    	return 0;
    }

    // Seteada la heuristica AQUI COMIENZA EL ALGORITMO A TRABAJAR, seteamos el punto de inicio y lo guardamos dentro de la estructura
    // correspondiente
    if(findStart(Gridcopy, parameters, ptrChanges)){
    	// Si estamos aqui es porque se encontro el punto de inicio con exito
    	__NOP();
    }else{

    	// Si estamos aqui no hay punto de inicio y no se puede realizar el algoritmo
    	return 0;
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

    	// Imprimimos la matriz a estudiar

    	// Imprimimos el mensaje de el estudio en el que estamos
    	sprintf (buffer,"\n__________ESTUDIO NUMERO %u__________\n", counterStudy);
    	writeMsg(&handlerAstarUsart, buffer);

    	// Imprimimos la matriz 3x3 a estudiar
    	for(i = 0; i < 3; i++){
			for (j = 0; j < 3; j++){
				writeChar(&handlerAstarUsart, nineSlotsMatriz[i][j]);
			}
			writeChar(&handlerAstarUsart, '\r');
    	}

    	writeChar(&handlerAstarUsart, '\r');

    	// Imprimimos el estado actual de la matriz copia
    	for (uint8_t i = 0; i< parameters->numberOfRows; i++){
    			writeMsg(&handlerAstarUsart, Gridcopy[i]);
		}
    	// Escribimos enter para tener todo bien espaciado
    	writeChar(&handlerAstarUsart, '\r');


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
						ptrChanges->Fcost = setFcost(parameters, ptrChanges, position, matrixCosts);
						// Estudiamos si el nuevo FCost es mayor menor o igual al Fcost que ya tiene el estado abierto
						if (ptrChanges->Fcost >= matrixCosts[ptrChanges->posAnalisis[0] + i - 1][ptrChanges->posAnalisis[1] + j - 1][1]){
							// Si el Fcost es mayor o igual a el anteriormente calculado, Significa que el Gcost calculado es el mismo o mayor,
							//por lo que no se actualiza ni el G cost y el F cost ni el parent
							__NOP();
						}else{
							// Si estamos aqui es porque el Fcost calculado para el G cost nuevo es mejor y se debe actualizar tanto el Gcost el Fcost
							// como el parent.
							updateGcost(parameters, ptrChanges, position, matrixCosts);
							updateFcost(parameters, ptrChanges, position, matrixCosts);
							updateParent(ptrChanges, position, matrixCosts);

							// Si si actualizamos la posicion abierta respectiva, tambien se debe actualizar en la matriz de decisión el F cost
							decisionMatrix[(int) matrixCosts[ptrChanges->posAnalisis[0] + i - 1][ptrChanges->posAnalisis[1] + j - 1][5]][0] =
									       matrixCosts[ptrChanges->posAnalisis[0] + i - 1][ptrChanges->posAnalisis[1] + j - 1][1]; // La segunda matriz son los F costs

						}
						break;
					}case '*':{
						// El séptimo caso seria cuando el puntero que estudia la matriz corresponde con un punto '*' que corresponde con un
						// espacio no estudiado, por lo que simplemente se setea sobre estos nuevos puntos su Gcost y su Fcost, incluyendo el parent
						position[0] = i;
						position[1] = j;
						updateGcost(parameters, ptrChanges, position, matrixCosts);
						updateFcost(parameters, ptrChanges, position, matrixCosts);
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
						return 0;
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
			findLesserValue(ptrChanges, decisionMatrix, counter);
			// A partir de aqui tendriamos la posicion del F cost mas pequeño, pero primero se compueba de que si depronto hubo un F cost igual
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

				//Actualizamos el punto de analisis con la posicion de la heuristica mas pequeña, usando la matriz de decision entregamos la posicion
				// respectiva que contiene la caracteristica deseada
				ptrChanges->posAnalisis[0] = decisionMatrix[ptrChanges->lesserHcostPosition][2]; // Posicion i del valor de la Heuristica mas corta
				ptrChanges->posAnalisis[1] = decisionMatrix[ptrChanges->lesserHcostPosition][3]; // Posicion j del valor de la heuristica mas corta

				// Limpiamos la posicion de la matriz de decision ya que este punto pasa a ser un valor a estudiar, y debe de salir de la lista
				// de estados en Open
				for (i = 0 ; i < 4 ; i++){
					decisionMatrix[ptrChanges->lesserHcostPosition][i] = 0;
				}

				//solo queda mover las posiciones en estado de abierto para arriba para que ocupen el espacio que quedo

				for (i = ptrChanges->lesserHcostPosition + 1; i < counter ; i++ ){
					for (j = 0 ; j < 4 ; j++){
						// Mudamos los elementos de la posicion i a la posicion anterior
						decisionMatrix[i - 1][j] = decisionMatrix[i][j];
						// Limpiamos la posicion que acabamos de mudar para mudar a la siguiente
						decisionMatrix[i][j] = 0;
					}
				}

				// Al final restamos uno al counter ya que de su lista salio un compañero
				counter--;

				//Colocamos en estado de open el nuevo estado a estudiar
				ptrChanges->posOpen[0] = ptrChanges->posAnalisis[0];
				ptrChanges->posOpen[1] = ptrChanges->posAnalisis[1];

				// Resetemos la bandera que nos indica si hay un valor de Fcost igual.
				ptrChanges->equalFcost = RESET;

			}else{
				// Si estamos aqui es porque no hubo F costs iguales y se hara lo mismo que en el caso anterior solo que se tendra en cuenta
				// la posicion encontrada del Fcost mas pequeño, ya no del Hcost mas pequeño
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

				//Actualizamos el punto de analisis con la posicion de la heuristica mas pequeña, usando la matriz de decision entregamos la posicion
				// respectiva que contiene la caracteristica deseada
				ptrChanges->posAnalisis[0] = decisionMatrix[ptrChanges->lesserFcostPosition][2]; // Posicion i del valor de la Heuristica mas corta
				ptrChanges->posAnalisis[1] = decisionMatrix[ptrChanges->lesserFcostPosition][3]; // Posicion j del valor de la heuristica mas corta

				// Limpiamos la posicion de la matriz de decision ya que este punto pasa a ser un valor a estudiar, y debe de salir de la lista
				// de estados en Open
				for (i = 0 ; i < 4 ; i++){
					decisionMatrix[ptrChanges->lesserFcostPosition][i] = 0;
				}

				//solo queda mover las posiciones en estado de abierto para arriba para que ocupen el espacio que quedo

				for (i = ptrChanges->lesserFcostPosition + 1; i < counter ; i++ ){
					for (j = 0 ; j < 4 ; j++){
						// Mudamos los elementos de la posicion i a la posicion anterior
						decisionMatrix[i - 1][j] = decisionMatrix[i][j];
						// Limpiamos la posicion que acabamos de mudar para mudar a la siguiente
						decisionMatrix[i][j] = 0;
					}
				}

				// Al final restamos uno al counter ya que de su lista salio un compañero
				counter--;

				//Colocamos en estado de open el nuevo estado a estudiar
				ptrChanges->posOpen[0] = ptrChanges->posAnalisis[0];
				ptrChanges->posOpen[1] = ptrChanges->posAnalisis[1];

			}

			counterStudy++;


    	}else{
			// Si el programa entra en esta condicion quiere decir que ya se encontro la ruta mas corta y ya es hora de construir la matriz de posiciones
			// donde se almacenara la ruta mas corta
			//buscamos cuantos elementos deberia de tener el arreglo para ello usaremos el siguiente while donde recorreremos desde el end hasta el
			//start
			i = ptrChanges->endPos[0];
			j = ptrChanges->endPos[1];
			while(Gridcopy[i][j] != 's'){
				// Actualizamos a la nueva posición
				position[0] = matrixCosts[i][j][3];
				position[1] = matrixCosts[i][j][4];

				// actualizamos la nueva posición
				i = position[0];
				j = position[1];

				// incrementamos en uno la cantidad de posiciones a guardar
				numberOfPositions++;
			}
		}


    }// final del ciclo While

    writeMsg(&handlerAstarUsart, "\n___________Hemos Encontrado la ruta mas corta______________\n");
    writeChar(&handlerAstarUsart, '\r');

    // estando aqui ya solo queda almacenar toda las posiciones parent comenzando desde el end hasta el start, siguendo el parent de cada uno se asegura
    // que lo que se esta almacenando es la ruta mas corta

    //Almacenamos dentro de una de las variables del arreglo AStar_distancesHandler la cantidad de elementos que tiene la matriz de la ruta mas corta
    parameters->numberOfElements = numberOfPositions + 1 ; // Le sumamos uno mas para incluir el punto de inicio


    i = ptrChanges->endPos[0];
	j = ptrChanges->endPos[1];

    // Recorremos la matriz e iremos almacenando dentro de este comenzando desde la posicion final y terminando en la posicion inicial
    for (int  k = numberOfPositions; k >= 0 ; k--){
    	//Cambiamos la matriz redeableGrid, las posiciones que corresponden a la ruta mas corta por un char 'I',
    	if (Gridcopy[i][j] == 'e'){
    		// Si estamos aqui es porque no queremos cambiar el char de finalización
    		__NOP();
    	}else if (Gridcopy[i][j] == 's'){
    		// Si estamos aqui es porque no queremos cambiar el char de inicio
    		__NOP();
    	}else{
    		// Si estamos aqui es porque estamos dentro del camino a seguir, por lo cambiamos a una 'I'
    		Gridcopy[i][j] = 'I';
    	}


    	// Almacenamos en la matriz de ruta mas corta
		shorterWay[k][0] = i;
		shorterWay[k][1] = j;

    	// comenzamos almacenando las posiciones en orden desde el final al punto inicial
		// Actualizamos a la nueva posición
		position[0] = matrixCosts[i][j][3];
		position[1] = matrixCosts[i][j][4];

		// actualizamos la nueva posición
		i = position[0];
		j = position[1];


    }
	// Imprimimos el estado actual de la matriz copia
	for (uint8_t i = 0; i< parameters->numberOfRows; i++){
			writeMsg(&handlerAstarUsart, Gridcopy[i]);
	}

    // A partir de aqui habremos logrado despues de un largo camino hallar la ruta mas corta entre dos puntos , el robot ya con esa informacion
    // sabra hacia donde moverse y cuanto moverse

    // Si llegamos hasta aca, con exito hemos logrado todo lo cometido, ¡Felicidades!
    writeMsg(&handlerAstarUsart, "\n______________________Gracias por viajar con nuestra linea Astarlines____________________\n");

    return 1;


}

// Esta función actuazliza en la matriz de costs y el parent correspondiente
void updateParent(costChangesAndPos_t *ptrChanges, int posIJ[2], float matrixCosts[7][7][6]){

	setParents(ptrChanges, posIJ);

	matrixCosts[ptrChanges->posAnalisis[0] + posIJ[0] - 1][ptrChanges->posAnalisis[1] + posIJ[1] - 1][3] = ptrChanges->parent[0]; //Posicion i del parent
	matrixCosts[ptrChanges->posAnalisis[0] + posIJ[0] - 1][ptrChanges->posAnalisis[1] + posIJ[1] - 1][4] = ptrChanges->parent[1]; //Posicion j del parent

}

// esta funcion actualiz el Gcost correspondiente
void updateGcost(AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges, int posIJ[2], float matrixCosts[7][7][6] ){
	//Por ultimo se setea en la pisicion de la heuristica correspondiente a la matriz ultima de la super matriz
	// de costos
	matrixCosts[ptrChanges->posAnalisis[0] + posIJ[0] -1][ptrChanges->posAnalisis[1] + posIJ[1] -1][0] = setGcost(parameters, ptrChanges, posIJ);
}

// Esta función actualiza el Fcost correspondiente
void updateFcost(AStar_distancesHandler *parameters ,costChangesAndPos_t *ptrChanges, int posIJ[2], float matrixCosts[7][7][6] ){
	//Por ultimo se setea en la pisicion de la heuristica correspondiente a la matriz ultima de la super matriz
	// de costos
	matrixCosts[ptrChanges->posAnalisis[0] + posIJ[0] -1][ptrChanges->posAnalisis[1] + posIJ[1] -1][1] = setFcost(parameters, ptrChanges, posIJ, matrixCosts);

}

// con esta funcion seteamos la matriz Heuristica con la cual usaremos la info para buscar la ruta mas corta
int setHeuristic(AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges, float matrixCosts[7][7][6] , char Gridcopy[7][7]){

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
	 && (ptrChanges->posAnalisis[1] + posIJ[1] -1) != ptrChanges->posAnalisis[1]){
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
float setFcost (AStar_distancesHandler *parameters , costChangesAndPos_t *ptrChanges, int posIJ[2], float matrixCosts[7][7][6]){

	// Esta funcion es simple ya que solo tenemos que calcular de la matriz 3x3 de analisis y sumar el H cost y el G cost para tener el F cost
	ptrChanges->Gcost = setGcost(parameters, ptrChanges, posIJ);
	ptrChanges->Fcost = ptrChanges->Gcost  // Gcost
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
int findStart(char Gridcopy[7][7], AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges){

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
int findEnd(char Gridcopy[7][7], AStar_distancesHandler *parameters, costChangesAndPos_t *ptrChanges){


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
//void buildArrayShorterWay(AStar_distancesHandler *parameters, int **shorterWayArray){
//
//	//Repartimos la memoria teniendo en cuenta el numero de elementos que corresponden a la ruta mas corta seguida, cada entrada
//	// almacenará las posiciones que debe de seguir el robot incluyendo el punto de inicio y final
//	shorterWayArray = (int ** ) malloc(parameters->numberOfElements * sizeof(int *));
//	for (uint8_t i = 0 ; i < parameters->numberOfElements; i++){
//		shorterWayArray [i] = (int *)malloc(2 *sizeof(int));
//	}
//
//}

//Con esta funcion se reparte la memoria para la matriz de entrada desde la terminal serial

void buildMatrixCopy(AStar_distancesHandler *parameters, char terminalGrid[7][7], char Gridcopy[7][7]){


	// Seteamos los valores dentro de la matriz infoGrid de la entrada respectiva
	for (uint8_t i = 0; i < parameters->numberOfRows; i++){
		for(uint8_t j = 0; j < parameters->numberOfColumns + 2; j++){

			if (j == parameters->numberOfColumns){
				// Agregamos al a la posicion penultima, agregamos una terminacion de salto de linea para ipresion en consola
				Gridcopy[i][j] = '\r';
			}else if (j == parameters->numberOfColumns + 1){
				// Agregamos al final la terminacion nula para que cada fila sea un string completo
				Gridcopy[i][j] = '\0';
			}else{
				Gridcopy[i][j] = terminalGrid[i][j];
			}
		}
	}

}

//Con esta funcion repartimos la memoria para la matriz de costos
//void buildMatrixCosts(AStar_distancesHandler *parameters, float ***matrixCosts){
//
//	matrixCosts = (float***) malloc(parameters->numberOfRows * sizeof(float**));
//	for (int i = 0; i < parameters->numberOfRows; i++) {
//		matrixCosts[i] = (float **)malloc(parameters->numberOfColumns * sizeof(float*));
//		for (int j = 0; j < parameters->numberOfColumns; i++) {
//			matrixCosts[i][j] = (float *)malloc(6 * sizeof(float));
//		}
//	}
//
//}



// Se define la funcion de tomar cantidad de filas recorriendo la cantidad de String que tenga el puntero de arreglos matrix hasta que se
// encuentre con el puntero nulo.
uint8_t getRows(char terminalGrid[7][7]){

	uint8_t counterRows = 0;
	char letter = '\0';
	(void)letter;
	while(terminalGrid[counterRows][0] != '\0'){
		letter = terminalGrid[counterRows][0];
		counterRows++;

	}

	return counterRows;
}

//Se define la funcion de tomar cantidad de columnas recorriendo el string hasta encontrar el elemento nulo char
uint8_t getColums(char terminalGrid[7][7]){

	uint8_t counterColumns = 0;
	while(terminalGrid[0][counterColumns] != '\0'){

		counterColumns++;

	}

	return counterColumns;
}

// esta funcion nos almacena en uno de los arrays volatiles de la estructura costChangesAndPos_t la posicion del valor Fcost o H cost mas pequeño,
// Se debe identificar con un string si se quiere hallar el Fcost mas pequeño o el Hcost mas pequeño, asi, "Fcost" si se quiere hallar el F cost o
// "Hcost" si se quiere hallar el H cost
void findLesserValue(costChangesAndPos_t *ptrChanges, float decisionMtrx[500][4], uint8_t contador){
	// seteamos las variables locales
	uint8_t i;
	uint8_t j;
	float value_1 = 0;
	float value_2 = 0;

	// counter es la entrada que nos representa la cantidad de posiciones regristradas en la matriz de decision hasta ahora


	// El algoritmo que se usará es que se recorrerá cada una de las posiciones y se analizara con las demas , excpliyendo obviamente
	// la posicion central


	for(i = 0; i<contador ; i++){
		for(j = 0; j<contador ; j++){
			if (i == j){
				// Si estamos aqui es porque estamos analizando el mismo punto, y nosotros queremos es analizar a sus compañeros solamente
				__NOP();
			}else if (decisionMtrx[i][0]
					< decisionMtrx[j][0]){
				// Si estamos aca es porque podemos hacer la comparación
				// Si estamos aca es porque efectivamente el Fcost es menor, Solo dejamos pasar el ciclo para asegurarnos de que el j
				// pueda llegar hasta su valor final permitido
			}else if (decisionMtrx[i][0]
				   == decisionMtrx[j][0]){
				// Si estamos aqui es porque el programa encontro mas de un  minimo un valor igual al analizado
				ptrChanges->equalFcost = SET;
			}else{
				// Si estamos aqui es porque el programa hallo almenos un valor menor al analizado, por lo que no nos sirve
				// Se resetea la bandera que decia que habia un vakor igual, esto debe de ser solo cierto si el valor que es
				// igual es el menor de la matriz
				ptrChanges->equalFcost = RESET;
				break;
			}

		}//Terminacion de un for
		if (j == contador){
			// Si entramos en esta condicion es porque el contador j logro llegar a su posicion final
			// Si entramos aqui es porque se logro analizar todo el arreglo y se encontro la posicion que corresponde con el Fcost mas pequeño
			// de todos, tambien guardamos el valor mas pequeño, si este se repite, se sabra gracias a la bandera
			// La posicion mas pequeña del F cost corresponde con la que se etsaba analizando en la posición i
			ptrChanges->lesserFcostPosition = i;
			// Almacenamos en lesserFcost el valor de ese Fcost mas pequeño que se hallo
			ptrChanges->lesserFcost = decisionMtrx[i][0];
			// paramos el primer for
			break;
		}

	}//Terminacion del otro for


	// Ahora hallaremos el valor mas pequeño de H cost, solo en el caso de que la bandera correspondiente se haya levantado
	if (ptrChanges->equalFcost){
		// Si estamos aqui es porque si hay mas de un valor de F cost que corresponde con el valor mas pequeño, desempatamos buscando el Hcost mas pequeño
		// Para ello recorreremos la matriz a analizar de nuevo pero esta vez solo buscando aquellos valores que correspondan con el valor hallado de Fcost

		for(i = 0; i<contador ; i++){
			value_1 = decisionMtrx[i][0] / ptrChanges->lesserFcost;
			for(j = 0; j<contador ; j++){
				value_2 = decisionMtrx[j][0] / ptrChanges->lesserFcost;
 				if (i == j){
					// Si estamos aqui es porque estamos analizando la posición de analisis, por lo que lo ignoramos
					__NOP();
				}else if (value_1 == 1 && value_2 == 1){
					// Si estamos aca es porque podemos hacer la comparación pero esta vez con la matriz heuristica
					if (decisionMtrx[i][1] <= decisionMtrx[j][1]){
						// Si estamos aca es porque efectivamente el Hcost es menor o igual al resto de Hcost
						__NOP();
					}else{
						// Si estamos aqui es porque el programa hallo almenos un valor menor al analizado, por lo que no nos sirve
						break;
					}
				}else if (value_1 == 1){
					// Si estamos aqui es porque aun podemos seguir comparando, por lo que el algoritmo debe de seguir bucando a sus iguales
					__NOP();
				}else {
					// Estamos en un valor que no nos interesa evaluar, por lo que lo ignoramos y salimos
					break;
				}

			}//Terminacion de un for
			if (j == contador){
				// Si entramos aqui es porque se logro analizar toda la matriz y se encontro la posicion que corresponde con la posicion mas pequeña
				// de todas, tambien guardamos el valor mas pequeño, es muy poco probable, por no decir imposible que tengamos una misma heuristica
				//repetida
				ptrChanges->lesserHcostPosition = i;
				//Almacenamos en lesserHcost el valor del Hcost mas pequeño encontrado
				ptrChanges->lesserHcost = decisionMtrx[i][1];
				// Paramos el primer for
				break;
			}

		}//Terminacion del otro for

	}else{
		// Si estamos aca es porque no se alzo la bandera que indica que hay mas de un F cost igual por lo que no hacemos nada
		__NOP();
	}

}

// Con esta funcion liberamos la memoria utilizada por la super matriz de costos
void freeCosts(AStar_distancesHandler *parameters, float***matrixCosts){

	// Liberamos cada columna
	for (int i = 0; i < parameters->numberOfRows; i++) {
		for (int j = 0; j < parameters->numberOfColumns; j++) {
			free(matrixCosts[i][j]);
		}
	}

	// Liberamos cada fila
	for (int i = 0; i < parameters->numberOfRows; i++) {
				free(matrixCosts[i]);
	}

	// Liberamos el arreglo de punteros
	free(matrixCosts);


}

// Con esta funcion liberamos la memoria utilizada por la matriz que almacena la copia de los strings ingresados
void freeReadableGrid(AStar_distancesHandler *parameters, char** Gridcopy){

	// Liberamos cada fila
	for (int i = 0; i < parameters->numberOfRows; i++) {
				free(Gridcopy[i]);
	}

	// Liberamos el arreglo de punteros
	free(Gridcopy);

}

// con esta funcion liberamos la memoria utilizada por la matriz que almacena la ruta mas corta
void freeShorterWay(AStar_distancesHandler *parameters, int **shorterWayArray){

	// Liberamos cada fila
	for (int i = 0; i < parameters->numberOfElements; i++) {
				free(shorterWayArray[i]);
	}

	// Liberamos el arreglo de punteros
	free(shorterWayArray);

}
// Con las siguientes funciones inicializamos a los handler necesarios para poder usar la comunicacion serial desde aqui y no desde el main
void initSerialComunication (USART_Handler_t *ptrHandlerUsart, GPIO_Handler_t *ptrHandlerRx, GPIO_Handler_t *ptrHandlerTx){

	// Inicializamos para el modulo Usart, no se necesita configurar ya que ya en el main se configuro con el handler específico
	handlerAstarUsart.ptrUSARTx                      = ptrHandlerUsart->ptrUSARTx;
	handlerAstarUsart.USART_Config.USART_MCUvelocity = ptrHandlerUsart->USART_Config.USART_MCUvelocity;
	handlerAstarUsart.USART_Config.USART_baudrate    = ptrHandlerUsart->USART_Config.USART_baudrate;
	handlerAstarUsart.USART_Config.USART_enableInTx  = ptrHandlerUsart->USART_Config.USART_enableInTx;
	handlerAstarUsart.USART_Config.USART_mode        = ptrHandlerUsart->USART_Config.USART_mode;
	handlerAstarUsart.USART_Config.USART_parity      = ptrHandlerUsart->USART_Config.USART_parity;
	handlerAstarUsart.USART_Config.USART_stopbits    = ptrHandlerUsart->USART_Config.USART_stopbits;
	handlerAstarUsart.USART_Config.USART_datasize    = ptrHandlerUsart->USART_Config.USART_datasize;

	// Hacemos lo mismo con los pines Rx y Tx del GPIO
	handlerAstarPinRx.pGPIOx                             = ptrHandlerRx->pGPIOx;
	handlerAstarPinRx.GPIO_PinConfig.GPIO_PinAltFunMode  = ptrHandlerRx->GPIO_PinConfig.GPIO_PinAltFunMode;
	handlerAstarPinRx.GPIO_PinConfig.GPIO_PinMode        = ptrHandlerRx->GPIO_PinConfig.GPIO_PinMode;
	handlerAstarPinRx.GPIO_PinConfig.GPIO_PinOPType      = ptrHandlerRx->GPIO_PinConfig.GPIO_PinOPType;
	handlerAstarPinRx.GPIO_PinConfig.GPIO_PinNumber      = ptrHandlerRx->GPIO_PinConfig.GPIO_PinNumber;
	handlerAstarPinRx.GPIO_PinConfig.GPIO_PinPuPdControl = ptrHandlerRx->GPIO_PinConfig.GPIO_PinPuPdControl;
	handlerAstarPinRx.GPIO_PinConfig.GPIO_PinSpeed       = ptrHandlerRx->GPIO_PinConfig.GPIO_PinSpeed;

	handlerAstarPinTx.pGPIOx                             = ptrHandlerTx->pGPIOx;
	handlerAstarPinTx.GPIO_PinConfig.GPIO_PinAltFunMode  = ptrHandlerTx->GPIO_PinConfig.GPIO_PinAltFunMode;
	handlerAstarPinTx.GPIO_PinConfig.GPIO_PinMode        = ptrHandlerTx->GPIO_PinConfig.GPIO_PinMode;
	handlerAstarPinTx.GPIO_PinConfig.GPIO_PinOPType      = ptrHandlerTx->GPIO_PinConfig.GPIO_PinOPType;
	handlerAstarPinTx.GPIO_PinConfig.GPIO_PinNumber      = ptrHandlerTx->GPIO_PinConfig.GPIO_PinNumber;
	handlerAstarPinTx.GPIO_PinConfig.GPIO_PinPuPdControl = ptrHandlerTx->GPIO_PinConfig.GPIO_PinPuPdControl;
	handlerAstarPinTx.GPIO_PinConfig.GPIO_PinSpeed       = ptrHandlerTx->GPIO_PinConfig.GPIO_PinSpeed;

	// Ya seteados estos handler en teoria podriamos mandar por terminarl serial desde este .c

}




// Con esta funcion se setea el vector de operaciones con el que podemos recorrer el camino sin problemas

void create_Astar_operations(AStar_distancesHandler *parameters,
					   int shorterWayArray[100][2],
					   Parameters_Operation_t prtList[30],
					   Parameter_build_t *ptrbuild,
					   Parameters_Path_t *ptrPath,
					   Parameters_Position_t *ptrPos){

	// Aqui lo que se tendra en cuenta es el listado de operaciones necesarias para recorrer el camino, un listado donde
	// solo sea recorrerlo en el main y leer cada operacion y simplemene usar goto y rollto para aplicar tales operaciones

	// Aqui se va a suponer que donde se resetea el robot es el (0,0) de coordenadas y que su vector directos esta a 0 grados con respecto
	// al eje X, por lo que se puede colocar el robot como sea en la posicion inicial y este calculara sus operaciones dependiendo
	// de su posicion inicial


	double finishline_x = 0;
	double finishline_y = 0;
	double dist_to_x = 0;
	double dist_to_y = 0;

	ptrPath->start_position_x = ptrbuild->initline_x = 0;
	ptrPath->start_position_y = ptrbuild->initline_y = 0; //posicion de start, considerada como (0,0)


	// calculo del vector unitario del robot


	unitary_vector(ptrPos->rad_global, ptrbuild->delta_before);




	for (uint8_t i = 0 ; i < parameters->numberOfElements - 1 ; i++){

		// Seteamos como punto inicial el punto de start y como punto final el siguiente punto a ir

		dist_to_y = -(shorterWayArray[i+1][0] - shorterWayArray[i][0]) * parameters->parallelDistance;
		dist_to_x = (shorterWayArray[i+1][1] - shorterWayArray[i][1]) * parameters->parallelDistance;

		finishline_x += dist_to_x; // Coordenada x a ir
		finishline_y += dist_to_y; // Coordenada y a ir

		ptrPath->goal_Position_x = finishline_x;
		ptrPath->goal_Position_y = finishline_y;

		build_Operation(prtList, ptrbuild, finishline_x, finishline_y); // Agregamos la operación respectiva ya sea si se tiene que rotar o si

//		change_coordinates_position(ptrPath, finishline_x, finishline_y); // Cambiamos de coordenada teorica para seguir construyendo el camino

	}


	// Agregamos la operacion nula
	add_Operation(prtList, ptrbuild->number_operation, NULL_OPERATION, 0, 0, 0);
	ptrbuild->number_operation = 0;


}
