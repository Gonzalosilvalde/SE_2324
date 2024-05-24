/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include "MKL46Z4.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SW1_PIN 3
#define SW2_PIN 12
#define LED_GREEN_PIN 5
#define LED_RED_PIN 29

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
// MACROS LED
#define LED_GREEN_ONN()   (GPIOD->PCOR = (1 << LED_GREEN_PIN)) // Encender LED verde
#define LED_GREEN_OFFF()  (GPIOD->PSOR = (1 << LED_GREEN_PIN)) // Apagar LED verde
#define LED_RED_ONN()     (GPIOE->PCOR = (1 << LED_RED_PIN))   // Encender LED rojo
#define LED_RED_OFFF()    (GPIOE->PSOR = (1 << LED_RED_PIN))   // Apagar LED rojo
// MACROS estado botones
#define SW1_PRESSED()    ((GPIOC->PDIR & (1 << SW1_PIN)) == 0) // Verificar si SW1 está presionado
#define SW2_PRESSED()    ((GPIOC->PDIR & (1 << SW2_PIN)) == 0) // Verificar si SW2 está presionado

// Definicion de un tipo binario para los valores [0,1,2,3]
typedef enum {
    BIN_00 = 0b00,  // 0 en binario
    BIN_01 = 0b01,  // 1 en binario
    BIN_10 = 0b10,  // 2 en binario
    BIN_11 = 0b11   // 3 en binario
} BinaryValue;

BinaryValue bin_var = BIN_11;//Como variable global funciona mejor
void controlar_LEDs() {
  switch (bin_var) {
    case BIN_00:
    case BIN_01:
    case BIN_10:
      LED_GREEN_OFFF();
      LED_RED_ONN();
      break;
    case BIN_11:
      LED_GREEN_ONN();
      LED_RED_OFFF();
      break;
    default:
      break;
  }
}

// Funcion para inicializar LEDs modificada para iniciarlos a la vez
void led_init() {
    SIM->COPC = 0; //watchdog
    // Inicialización del LED verde
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    PORTD->PCR[LED_GREEN_PIN] = PORT_PCR_MUX(1);
    GPIOD->PDDR |= (1 << LED_GREEN_PIN);
    LED_GREEN_OFFF(); // Apagar LED verde inicialmente  
    // Inicialización del LED rojo
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    PORTE->PCR[LED_RED_PIN] = PORT_PCR_MUX(1);
    GPIOE->PDDR |= (1 << LED_RED_PIN);
    LED_RED_OFF(); // Apagar LED rojo inicialmente
}

// Funcion para inicializar los botones
void sw_init() {
    SIM->COPC = 0;//watchdog
    // Inicializacion del boton sw1
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; 
    PORTC->PCR[SW1_PIN] = PORT_PCR_MUX(1);  
    GPIOC->PDDR |= (1 << SW1_PIN); 
    GPIOC->PSOR = (1 << SW1_PIN);
    PORTC->PCR[SW1_PIN] |= PORT_PCR_IRQC(0xA); // IRQ en el flanco de bajada
    // Inicializacion del boton sw2
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; 
    PORTC->PCR[SW2_PIN] = PORT_PCR_MUX(1);  
    GPIOC->PDDR |= (1 << SW2_PIN); 
    GPIOC->PSOR = (1 << SW2_PIN);
    PORTC->PCR[SW2_PIN] |= PORT_PCR_IRQC(0xA); // IRQ en el flanco de bajada
	NVIC_SetPriority(31, 0);	// Bajar prioridad
	NVIC_EnableIRQ(31);			// Activa la interrupcion
  controlar_LEDs();

}

void init_all(void){
    /* Init board hardware. */
    
    led_init();
    sw_init();

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    PRINTF("\r\nReinicio!\r\n");

}



BinaryValue cambiarSegundoBit(BinaryValue valor) {
    return valor ^ 0b10; // XOR con 0b10 para cambiar solo el segundo bit
}

// Función para cambiar solo el primer bit (bit 0) del valor de BinaryValue
BinaryValue cambiarPrimerBit(BinaryValue valor) {
    return valor ^ 0b01; // XOR con 0b01 para cambiar solo el primer bit
}

void PORTC_PORTD_IRQHandler(void) {
	PORTC->ISFR = 0xFFFFFFFF;	// Clear IRQ
    
    if (SW2_PRESSED()) {
        PORTC->PCR[SW1_PIN] |= PORT_PCR_ISF_MASK;
        
        bin_var= cambiarPrimerBit(bin_var);
        while (SW1_PRESSED()); // Espera hasta que se libere el botón

    }
    if (SW1_PRESSED()) {
        PORTC->PCR[SW2_PIN] |= PORT_PCR_ISF_MASK;
        bin_var=cambiarSegundoBit(bin_var);
        
        PRINTF("bin_var: %d\r\n", bin_var);

        while (SW2_PRESSED()); // Espera hasta que se libere el botón
    }
    controlar_LEDs();

}



/*!
 * @brief Main function
 */

int main(void)
{
  //char ch;

  /* Init board hardware. */
  init_all();


  while (1)
    {

    }
}
