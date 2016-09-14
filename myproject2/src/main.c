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
    	  digitalWrite( LED1, ON);
	   }
      else{
    	  digitalWrite( LED1, OFF);
	   }
   }
   return receivedByte;
}

void UART2_IRQHandler(void){
   uartWriteByte( uartReadByte() );
}

/* Set up and initialize board hardware */

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
