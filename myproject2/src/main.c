/* Copyright 2016, Eric Pernia.
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Main example source file
 **
 ** This is a mini example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Main example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * ENP          Eric Pernia
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 2016-04-26   v0.0.1   First version
 */

/*==================[inclusions]=============================================*/

#include "main.h"         /* <= own header */

#include "sAPI.h"         /* <= sAPI header */

#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif

#include "stdint.h"
#include "string.h"

/*==================[macros and definitions]=================================*/
#define TEC1_P    1
#define TEC1_P_   0
#define TEC1_GPIO 0
#define TEC1_PIN  4

#define TEC2_P    1
#define TEC2_P_   1
#define TEC2_GPIO 0
#define TEC2_PIN  8

#define TEC3_P    1
#define TEC3_P_   2
#define TEC3_GPIO 0
#define TEC3_PIN  9

#define TEC4_P    1
#define TEC4_P_   6
#define TEC4_GPIO 1
#define TEC4_PIN  9

#define LEDR_P    2
#define LEDR_P_   0
#define LEDR_GPIO 5
#define LEDR_PIN  0

#define LEDG_P    2
#define LEDG_P_   1
#define LEDG_GPIO 5
#define LEDG_PIN  1

#define LEDB_P    2
#define LEDB_P_   2
#define LEDB_GPIO 5
#define LEDB_PIN  2

#define LED1_P    2
#define LED1_P_  10
#define LED1_GPIO 0
#define LED1_PIN 14

#define LED2_P    2
#define LED2_P_  11
#define LED2_GPIO 1
#define LED2_PIN 11

#define LED3_P    2
#define LED3_P_  12
#define LED3_GPIO 1
#define LED3_PIN 12

#define INPUT     0
#define OUTPUT    1

#define ON        1
#define OFF       0

#define TICKRATE_HZ (1000) /* 1000 ticks per second --> 1ms Tick */

#define UART_USB      LPC_USART2  /* UART2 (USB-UART) */
#define BAUD_RATE     115200
#define UART2_TXD_P   7
#define UART2_TXD_P_  1
#define UART2_RXD_P   7
#define UART2_RXD_P_  2

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void boardButtonsInit(void) {

   /* Config EDU-CIAA-NXP Button Pins as GPIOs */
   Chip_SCU_PinMux(TEC1_P, TEC1_P_, MD_PUP|MD_EZI|MD_ZI, FUNC0); /* P1_0,  GPIO0[4], TEC1 */
   Chip_SCU_PinMux(TEC2_P, TEC2_P_, MD_PUP|MD_EZI|MD_ZI, FUNC0); /* P1_1,  GPIO0[8], TEC2 */
   Chip_SCU_PinMux(TEC3_P, TEC3_P_, MD_PUP|MD_EZI|MD_ZI, FUNC0); /* P1_2,  GPIO0[9], TEC3 */
	Chip_SCU_PinMux(TEC4_P, TEC4_P_, MD_PUP|MD_EZI|MD_ZI, FUNC0); /* P1_6,  GPIO1[9], TEC4 */

   /* Config EDU-CIAA-NXP Button Pins as Inputs */
   Chip_GPIO_SetDir(LPC_GPIO_PORT, TEC1_GPIO, (1<<TEC1_PIN), INPUT);
   Chip_GPIO_SetDir(LPC_GPIO_PORT, TEC2_GPIO, (1<<TEC3_PIN), INPUT);
   Chip_GPIO_SetDir(LPC_GPIO_PORT, TEC2_GPIO, (1<<TEC2_PIN), INPUT);
   Chip_GPIO_SetDir(LPC_GPIO_PORT, TEC4_GPIO, (1<<TEC4_PIN), INPUT);

}

static void boardLedsInit(void) {

   /* Config EDU-CIAA-NXP Led Pins as GPIOs */
   Chip_SCU_PinMux(LEDR_P, LEDR_P_, MD_PUP, FUNC4); /* P2_0,  GPIO5[0],  LEDR */
   Chip_SCU_PinMux(LEDG_P, LEDG_P_, MD_PUP, FUNC4); /* P2_1,  GPIO5[1],  LEDG */
   Chip_SCU_PinMux(LEDB_P, LEDB_P_, MD_PUP, FUNC4); /* P2_2,  GPIO5[2],  LEDB */
   Chip_SCU_PinMux(LED1_P, LED1_P_, MD_PUP, FUNC0); /* P2_10, GPIO0[14], LED1 */
   Chip_SCU_PinMux(LED2_P, LED2_P_, MD_PUP, FUNC0); /* P2_11, GPIO1[11], LED2 */
   Chip_SCU_PinMux(LED3_P, LED3_P_, MD_PUP, FUNC0); /* P2_12, GPIO1[12], LED3 */

   /* Config EDU-CIAA-NXP Led Pins as Outputs */
   Chip_GPIO_SetDir(LPC_GPIO_PORT, LEDR_GPIO, (1<<LEDR_PIN), OUTPUT);
   Chip_GPIO_SetDir(LPC_GPIO_PORT, LEDG_GPIO, (1<<LEDG_PIN), OUTPUT);
   Chip_GPIO_SetDir(LPC_GPIO_PORT, LEDB_GPIO, (1<<LEDB_PIN), OUTPUT);
   Chip_GPIO_SetDir(LPC_GPIO_PORT, LED1_GPIO, (1<<LED1_PIN), OUTPUT);
   Chip_GPIO_SetDir(LPC_GPIO_PORT, LED2_GPIO, (1<<LED2_PIN), OUTPUT);
   Chip_GPIO_SetDir(LPC_GPIO_PORT, LED3_GPIO, (1<<LED3_PIN), OUTPUT);

   /* Init EDU-CIAA-NXP Led Pins OFF */
   Chip_GPIO_ClearValue(LPC_GPIO_PORT, LEDR_GPIO, (1<<LEDR_PIN));
   Chip_GPIO_ClearValue(LPC_GPIO_PORT, LEDG_GPIO, (1<<LEDG_PIN));
   Chip_GPIO_ClearValue(LPC_GPIO_PORT, LEDB_GPIO, (1<<LEDB_PIN));
   Chip_GPIO_ClearValue(LPC_GPIO_PORT, LED1_GPIO, (1<<LED1_PIN));
   Chip_GPIO_ClearValue(LPC_GPIO_PORT, LED2_GPIO, (1<<LED2_PIN));
   Chip_GPIO_ClearValue(LPC_GPIO_PORT, LED3_GPIO, (1<<LED3_PIN));

}

static void coreInit(void) {

   /* Read clock settings and update SystemCoreClock variable */
   SystemCoreClockUpdate();

   /* To configure the Systick timer we use the SysTick_Config(uint32_t ticks);
    * funtion. With ticks we can set the Systick timer interval. In our case we
    * have a 204 MHz clock and we want it to fire each ms. So the tick has to
    * be set to 204 MHz / 1000.
    */
   SysTick_Config( SystemCoreClock / TICKRATE_HZ); /* TICKRATE_HZ ticks per second */
}

/* blocks for dlyTicks ms */
__INLINE static void delay(uint32_t dlyTicks){
    uint32_t curTicks;

    curTicks = msTicks;
    while ((msTicks - curTicks) < dlyTicks);
}

/*==================[external functions definition]==========================*/


__attribute__ ((section(".after_vectors")))
void SysTick_Handler(void) {
	msTicks++;
}

void uartConfig(uint32_t baudRate){

   Chip_UART_Init(UART_USB);
   Chip_UART_SetBaud(UART_USB, baudRate);  /* Set Baud rate */
   Chip_UART_SetupFIFOS(UART_USB, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0); /* Modify FCR (FIFO Control Register)*/
   Chip_UART_TXEnable(UART_USB); /* Enable UART Transmission */
   Chip_SCU_PinMux(UART2_TXD_P, UART2_TXD_P_ , MD_PDN, FUNC6);              /* P7_1,FUNC6: UART2_TXD */
   Chip_SCU_PinMux(UART2_RXD_P, UART2_RXD_P_ , MD_PLN|MD_EZI|MD_ZI, FUNC6); /* P7_2,FUNC6: UART2_RXD */

   /* Enable UART Rx Interrupt */
   Chip_UART_IntEnable(UART_USB, UART_IER_RBRINT ); /* Receiver Buffer Register Interrupt */
   /* Enable UART line status interrupt */
   Chip_UART_IntEnable(UART_USB, UART_IER_RLSINT ); /* LPC43xx User manual page 1118 */
   NVIC_SetPriority(USART2_IRQn, 6);
   /* Enable Interrupt for UART channel */
   NVIC_EnableIRQ(USART2_IRQn);

}

void uartWriteByte(uint8_t byte){

   while ((Chip_UART_ReadLineStatus(UART_USB) & UART_LSR_THRE) == 0); /* Wait for space in FIFO */
   Chip_UART_SendByte(UART_USB, byte);
}

void uartWrite(uint8_t * str){

   while(*str != 0){
	  uartWriteByte(*str);
	  *str++;
   }
}

uint8_t uartReadByte(void){

   uint8_t receivedByte = 0;

   if (Chip_UART_ReadLineStatus(UART_USB) & UART_LSR_RDR) {
      receivedByte = Chip_UART_ReadByte(UART_USB);
      if(receivedByte=='1'){				//DENTRO DE SUBRUTINA DE LECTURA??
      		   Chip_GPIO_SetValue(LPC_GPIO_PORT, LED2_GPIO, (1<<LED2_PIN));
	   }
      else{
      		   Chip_GPIO_ClearValue(LPC_GPIO_PORT, LED2_GPIO, (1<<LED2_PIN));
	   }
   }
   return receivedByte;
}

void UART2_IRQHandler(void){
   uartWriteByte( uartReadByte() );
}

/* Set up and initialize board hardware */
void boardInit(void) {

   /* Config Core */
   coreInit();

   /* Initializes GPIO */
   Chip_GPIO_Init(LPC_GPIO_PORT);

   /* Config EDU-CIAA-NXP Button Pins */
   boardButtonsInit();

   /* Config EDU-CIAA-NXP Led Pins */
   boardLedsInit();

}


 /* FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE RESET. */
int main(void)
{
   /* ------------- INICIALIZACIONES ------------- */

   /* Inicializar la placa */
   boardConfig();

   /* Inicializar el conteo de Ticks con resolución de 1ms */
   tickConfig(1,0);

   /* Inicializar DigitalIO */
   digitalConfig( 0, ENABLE_DIGITAL_IO );

   /* Configuración de pines de entrada para
	   Teclas de la CIAA-NXP */
   digitalConfig( TEC1, INPUT );
   digitalConfig( TEC2, INPUT );
   digitalConfig( TEC3, INPUT );
   digitalConfig( TEC4, INPUT );

   /* Configuración de pines de salida para
	   Leds de la CIAA-NXP */
   digitalConfig( LEDR, OUTPUT );
   digitalConfig( LEDG, OUTPUT );
   digitalConfig( LEDB, OUTPUT );
   digitalConfig( LED1, OUTPUT );
   digitalConfig( LED2, OUTPUT );
   digitalConfig( LED3, OUTPUT );

   /* ------------- REPETIR POR SIEMPRE ------------- */
	while(1) {
		while((digitalRead(TEC1)==OFF)&&(digitalRead(TEC2)==OFF)){
			digitalWrite( LEDB, ON);
			digitalWrite( LED1, ON);
			delay(1000);
			digitalWrite( LEDB, OFF);
			digitalWrite( LED1, OFF);
		}
		while((digitalRead(TEC3)==OFF)&&(digitalRead(TEC4)==OFF)){
			digitalWrite( LED2, ON);
			digitalWrite( LED3, ON);
			delay(1000);
			digitalWrite( LED2, OFF);
			digitalWrite( LED3, OFF);
		}
		digitalWrite( LEDB, OFF);
		digitalWrite( LED1, OFF);
		digitalWrite( LED2, OFF);
		digitalWrite( LED3, OFF);

	}

	/* NO DEBE LLEGAR NUNCA AQUI, debido a que a este
	   programa no es llamado por ningun S.O. */
	return 0 ;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
