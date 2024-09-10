/*
 * PosRobt.c
 *
 *  Created on: Sep 8, 2024
 *      Author: juan
 */


#include "PosRobt.h"
#include <math.h>


//-------------------------Funcion para la definicion de operaciones--------------------------
void build_Operation(Parameters_Operation_t *prtList, Parameter_build_t *prtbuild, double finishline_x, double finishline_y)
{
	//Definicion el vector director
	double delta[2] = {finishline_x - prtbuild->initline_x, finishline_y - prtbuild->initline_y};
	//Calculo angulo entre vectores directores
	double grad_turn_res = calculed_ang_turn(prtbuild->delta_before, delta);
	//condicional de cambio de angulo
	if(grad_turn_res == 0)
	{
		//agregar operacion de linea recta
		 add_Operation(prtList, prtbuild->number_operation, LINE, finishline_x, finishline_y, 0);
	}
	else
	{
		//Agregar operacion de rotacion
		prtbuild->number_operation++;
		add_Operation(prtList, prtbuild->number_operation, TURN, 0, 0, grad_turn_res);
		//agregar operacion de linea recta
		prtbuild->number_operation++;
		add_Operation(prtList, prtbuild->number_operation, LINE, finishline_x, finishline_y, 0);
	}
	//Se redefine los valores iniciales
	prtbuild->delta_before[0] = delta[0];
	prtbuild->delta_before[1] = delta[1];
	prtbuild->initline_x = finishline_x;
	prtbuild->initline_y = finishline_y;
}

void add_Operation(Parameters_Operation_t *prtList, uint8_t num_operation, uint8_t type_operation, double coor_x, double coor_y, double grad_turn)
{
	prtList[num_operation].operacion = type_operation;
	prtList[num_operation].x_destination = coor_x;
	prtList[num_operation].y_destination = coor_y;
	prtList[num_operation].grad_Rotative = grad_turn;
}


//-----------------Funciones para definir los parametros de la poscion teorica--------------
void change_position(Parameters_Path_t *ptrParameterPath, int distance)
{
	//Definimos la distancia
	 ptrParameterPath->line_Distance = distance;                  //[mm]
	//Calculamos la posicicion
	double pot_x = ptrParameterPath->line_Distance*cos((ptrParameterPath->rotative_Grad*M_PI)/180);
	double pot_y = ptrParameterPath->line_Distance*sin((ptrParameterPath->rotative_Grad*M_PI)/180);
	//Guardamos la posicion del Goal como la posicion de Start
	ptrParameterPath->start_position_x = ptrParameterPath->goal_Position_x;
	ptrParameterPath->start_position_y = ptrParameterPath->goal_Position_y;
	//Definimos la nueva posicion de llegada
	ptrParameterPath->goal_Position_x += pot_x;
	ptrParameterPath->goal_Position_y += pot_y;
}

void change_coordinates_position(Parameters_Path_t *ptrParameterPath, double coor_x, double coor_y)
{
	//Guardamos la posicion del Goal como la posicion de Start
	ptrParameterPath->start_position_x = ptrParameterPath->goal_Position_x;
	ptrParameterPath->start_position_y = ptrParameterPath->goal_Position_y;
	//Definimos la nueva posicion de llegada
	ptrParameterPath->goal_Position_x = coor_x;
	ptrParameterPath->goal_Position_y = coor_y;
	//Definimos la distancia
	ptrParameterPath->line_Distance = sqrt(pow((ptrParameterPath->goal_Position_x - ptrParameterPath->start_position_x),2)+
			pow(ptrParameterPath->goal_Position_y - ptrParameterPath->start_position_y,2));;                  //[mm]

}

//-------------Funcion para calcular los parametros del calculo de la distancia---------------
void calculation_parameter_distance(Parameters_Path_t  *ptrParameterPath)
{
	//Calculo del vector director de la recta

	// Para hacer un buen control al robot y que este pueda seguir de una forma muy bien aproximada la trayectoria deseada teorica es necesario
	// Aplicar geometria vectorial para calcular: LA DISTANCIA RECORRIDA SOBRE LA TRAYECTORIA TEORICA, Y LA DISTANCIA DESDE EL ROBOT A LA RECTA TEORICA.
	// A este ultimo es deseable calcularlo ya que queremos que esta distancia sea la mas pequeña posible cuando el robot este caminando.

	// las ecuaciones respectivas para hallar la proyeccion del vector del robot con respecto a su punto inicial SON:

	// |PROY_D^(Pos actual global - pos inicial)| = |(pos actual global - pos inicial) dot (pos final - pos inicial)/magnitud del vector director|
	// resolviendo esta ecuacion se llega a  =  [(pos final x- pos inicial x) * pos actual global x  + (pos final y- pos inicial y) * pos actual global y] / magnitud del director

	// Ahora para la distancia del robot a la recta se usa un producto cruz como herramienta, llegando a la ecuación simple para la distancia de:

	//  |{PROY_D^(Pos actual global - pos inicial) X (pos actual global - pos inicial) }|/|PROY_D^(Pos actual global - pos inicial)|

	// Obteniendo en el desarrollo como resultado:
	// -((pos final y - pos inicial y) * pos actual global x - (pos final x - pos inicial x) * pos actual global y)/ magnitud del director



	double director_x = ptrParameterPath->goal_Position_x - ptrParameterPath->start_position_x;
	double director_y = ptrParameterPath->goal_Position_y - ptrParameterPath->start_position_y;
	//definicion de la magnitud de director
	ptrParameterPath->magnitude_director = sqrt(pow(director_x,2)+pow(director_y,2));
	//definicion de los parametros para el calculo de la magnitud de la proyeccion sobre la recta
	ptrParameterPath->proy_Parte_1 = director_x;
	ptrParameterPath->proy_Parte_2 = director_y;
	ptrParameterPath->proy_Parte_1_2 = -1 * director_x * ptrParameterPath->start_position_x - director_y * ptrParameterPath->start_position_y;
	//definicion de los parametros para el calculo de la distancia del punto a la recta
	ptrParameterPath->dis_point_Parte_1 = director_y;
	ptrParameterPath->dis_point_Parte_2 = -1 * director_x;
	ptrParameterPath->dis_point_Parte_1_2 = -1 * director_y * ptrParameterPath->start_position_x + director_x * ptrParameterPath->start_position_y;
}

//------------Funciones para el calculo de la respectiva distancia------------------------
double distance_to_straight_line(Parameters_Path_t  *ptrParameterPath, double position_x, double position_y)
{
	// Calculo de la distancia del robot a la linea recta usando los parametros
	double distance = -1*((ptrParameterPath->dis_point_Parte_1 * position_x + ptrParameterPath->dis_point_Parte_2 * position_y
				       + ptrParameterPath->dis_point_Parte_1_2) / ptrParameterPath->magnitude_director);

	return distance;
}

double distance_traveled(Parameters_Path_t  *ptrParameterPath, double position_x, double position_y)
{
	//Calculo de la distancia viajada relativo a la linea recta
	double distance_Tra = (ptrParameterPath->proy_Parte_1 * position_x + ptrParameterPath->proy_Parte_2 * position_y
			+ ptrParameterPath->proy_Parte_1_2)/ptrParameterPath->magnitude_director;
	//Retornar
	return distance_Tra;
}

//------------------------------Funciones auxiliares-----------------------------------

double calculed_ang_turn(double vector_a[2], double vector_b[2])
{
    //Calculo de los diferentes elementos
    double dot = vector_a[0]*vector_b[1]-vector_b[0]*vector_a[1];
    double magvector_a = sqrt(pow(vector_a[0],2)+pow(vector_a[1],2));
    double magvector_b = sqrt(pow(vector_b[0],2)+pow(vector_b[1],2));
    double ang_between_vector = acos((vector_a[0]*vector_b[0]+vector_a[1]*vector_b[1])/(magvector_b*magvector_a));
    //conversion a grados
    ang_between_vector = (ang_between_vector*180) / M_PI;
    //agregamos la direccion de giro
    if(dot<0){ ang_between_vector = -ang_between_vector;}
    //Retornar valor
    return ang_between_vector;
}