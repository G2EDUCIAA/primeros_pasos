/**
*	\file funciones.c
*	\brief 
*	\details Descripcion detallada del archivo
*	\author FACUNDO
*	\date 29-09-2016 20:56:05
*/



#include "funciones.h" 
#include "sAPI.h"         /* <= sAPI header */
//Implementacion Switch-Case

/**
*	\fn void maquina_estado()
*	\brief Implementacion Switch-Case
*	\details 
*	\author FACUNDO
*	\date 29-09-2016 20:56:05
*/
void maquina_estado()
{
		static int estado = INICIAL;

		switch(estado)
		{
			case INICIAL:
			
				if(PRESS())
				{
					LED_R();
					estado = ROJO;
				}
				break;
			
			case ROJO:
				if(PRESS())
				{
					LED_1();
					estado = LED1;
				}
				break;
			
			case LED1:
			
				if(PRESS())
				{
					LED_2();
					estado = LED2;
				}

				break;
			
			case LED2:
			
				if(PRESS())
				{
					LED_3();
					estado = LED3;
				}

				break;
			
			case LED3:
			
				if(PRESS_7())
				{
					LED_G();
					estado = ROJO;
				}
				if(PRESS_4())
				{
					LED_B();
					estado = ROJO;
				}
				if(PRESS_10())
				{
					LED_R();
					BORROW();
					estado = ROJO;
		
				}

				break;
			
			default: estado = INICIAL;
		}


}

//Funciones asociadas a los eventos

/**
*	\fn int PRESS(void)
*	\brief Resumen
*	\details Detalles
*	\author FACUNDO
*	\date 29-09-2016 20:56:05
*/
int PRESS(void)
{
	int res = 0 ;
	//Codigo propio de la funcion
	if(digitalRead(DIO32)==OFF){
		res=1;
		cont++;  //incremento contador
	}
	return res;
}

/**
*	\fn int PRESS_4(void)
*	\brief Si el usuario "realiza primer pasada sobre los 4 leds" se prende led R
*	\details Detalles
*	\author FACUNDO
*	\date 29-09-2016 20:56:05
*/
int PRESS_4(void)
{
	int res = 0 ;
	//Codigo propio de la funcion
	if(cont==4){
		res=1;
	}
	return res;
}

/**
*	\fn int PRESS_7(void)
*	\brief Si el usuario "realiza segunda pasada sobre los 4 leds" se prende led B
*	\details Detalles
*	\author FACUNDO
*	\date 29-09-2016 20:56:05
*/
int PRESS_7(void)
{
	int res = 0 ;
	//Codigo propio de la funcion
	if (cont==7){
		res=1;
	}
	return res;
}

/**
*	\fn int PRESS_10(void)
*	\brief Si el usuario "realiza tercer pasada sobre los 4 leds" se prende led G
*	\details Detalles
*	\author FACUNDO
*	\date 29-09-2016 20:56:05
*/
int PRESS_10(void)
{
	int res = 0 ;
	//Codigo propio de la funcion
	if(cont==10){
		res=1;
	}
	return res;

}

//Funciones asociadas a las acciones

/**
*	\fn void LED_R(void)
*	\brief Resumen
*	\details Detalles
*	\author FACUNDO
*	\date 29-09-2016 20:56:05
*/
void LED_R(void)
{
	//Codigo propio de la funcion
	digitalWrite(LEDR,ON);
	digitalWrite(LED1,OFF);
	digitalWrite(LED2,OFF);
	digitalWrite(LED3,OFF);
}

/**
*	\fn void LED1(void)
*	\brief Resumen
*	\details Detalles
*	\author FACUNDO
*	\date 29-09-2016 20:56:05
*/
void LED1(void)
{
	//Codigo propio de la funcion
	digitalWrite(LEDR,OFF);
	digitalWrite(LED1,ON);
	digitalWrite(LED2,OFF);
	digitalWrite(LED3,OFF);
}

/**
*	\fn void LED2(void)
*	\brief Resumen
*	\details Detalles
*	\author FACUNDO
*	\date 29-09-2016 20:56:05
*/
void LED2(void)
{
	//Codigo propio de la funcion
	digitalWrite(LEDR,OFF);
	digitalWrite(LED1,OFF);
	digitalWrite(LED2,ON);
	digitalWrite(LED3,OFF);
}

/**
*	\fn void LED3(void)
*	\brief Resumen
*	\details Detalles
*	\author FACUNDO
*	\date 29-09-2016 20:56:05
*/
void LED3(void)
{
	//Codigo propio de la funcion
	digitalWrite(LEDR,OFF);
	digitalWrite(LED1,OFF);
	digitalWrite(LED2,OFF);
	digitalWrite(LED3,ON);
}

/**
*	\fn void LED_G(void)
*	\brief Resumen
*	\details Detalles
*	\author FACUNDO
*	\date 29-09-2016 20:56:05
*/
void LED_G(void)
{
	//Codigo propio de la funcion
	digitalWrite(LEDG,ON);
	digitalWrite(LED1,OFF);
	digitalWrite(LED2,OFF);
	digitalWrite(LED3,OFF);
}

/**
*	\fn void LED_B(void)
*	\brief Resumen
*	\details Detalles
*	\author FACUNDO
*	\date 29-09-2016 20:56:05
*/
void LED_B(void)
{
	//Codigo propio de la funcion
	digitalWrite(LEDB,ON);
	digitalWrite(LED1,OFF);
	digitalWrite(LED2,OFF);
	digitalWrite(LED3,OFF);
}


/**
*	\fn void BORROW(void)
*	\brief Renueva contador
*	\details Detalles
*	\author FACUNDO
*	\date 29-09-2016 20:56:05
*/
void BORROW(void)
{
	//Codigo propio de la funcion
	if (cont>=10){
		cont=0;
	}
}

