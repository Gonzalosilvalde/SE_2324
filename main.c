#include "MKL46Z4.h"
// Definicion de constantes
#define LED_GREEN_PIN 5
#define LED_RED_PIN 29
#define SW1_PIN 3
#define SW2_PIN 12

// MACROS para ahorrar espacion en ejecutable y reducir sobrecarga
// MACROS LED
#define LED_GREEN_ON()   (GPIOD->PCOR = (1 << LED_GREEN_PIN)) // Encender LED verde
#define LED_GREEN_OFF()  (GPIOD->PSOR = (1 << LED_GREEN_PIN)) // Apagar LED verde
#define LED_RED_ON()     (GPIOE->PCOR = (1 << LED_RED_PIN))   // Encender LED rojo
#define LED_RED_OFF()    (GPIOE->PSOR = (1 << LED_RED_PIN))   // Apagar LED rojo

// MACROS estado botones
#define SW1_PRESSED()    ((GPIOC->PDIR & (1 << SW1_PIN)) == 0) // Verificar si SW1 está presionado
#define SW2_PRESSED()    ((GPIOC->PDIR & (1 << SW2_PIN)) == 0) // Verificar si SW2 está presionado

// LED (RG)
// LED_GREEN = PTD5
// LED_RED = PTE29

void delay(void){
  volatile int i;

  for (i = 0; i < 100000; i++);
}
// Definicion de un tipo binario para los valores [0,1,2,3]
typedef enum {
    BIN_00 = 0b00,  // 0 en binario
    BIN_01 = 0b01,  // 1 en binario
    BIN_10 = 0b10,  // 2 en binario
    BIN_11 = 0b11   // 3 en binario
} BinaryValue;
// Funcion para aumentar en 1 el valor de BinaryValue. Si es 11, pasa 00.
BinaryValue incrementar(BinaryValue valor) {
    return (valor + 1) & 0b11; // Incrementa el valor en 1 y luego aplica una máscara para mantener solo los dos bits menos significativos,
                              // asegurando que el resultado esté en el rango de 0 a 3.
}
// Funcion para intercambuar valores de la variable BinaryValue. Si es 11, pasa 00, si es 01 pasa a 10, y viceversa 
BinaryValue intercambiar(BinaryValue valor) {
    return (valor ^ BIN_11) ^ (BIN_11 & BIN_00); // Realiza una operación XOR entre el valor y el binario 11, luego aplica otra operación XOR 
                                                // entre el resultado anterior y la operación AND entre el binario 11 y el binario 00. 
                                               // Esta función invierte los dos bits menos significativos del valor.
}
// Funcion para encender y apagar los LEDs dependiendo del estado de BinaryValue
void controlar_LEDs(BinaryValue bin_var) {
    // LED Rojo
    if (bin_var & BIN_01) {
        LED_RED_ON();  // Enciende el LED rojo
    } else {
        LED_RED_OFF();  // Apaga el LED rojo
    }
    // LED Verde
    if (bin_var & BIN_10) {
        LED_GREEN_ON();  // Enciende el LED verde
    } else {
        LED_GREEN_OFF();  // Apaga el LED verde
    }
}
// Funcion para inicializar LEDs modificada para iniciarlos a la vez
void led_init() {
    SIM->COPC = 0; //watchdog
    // Inicialización del LED verde
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    PORTD->PCR[LED_GREEN_PIN] = PORT_PCR_MUX(1);
    GPIOD->PDDR |= (1 << LED_GREEN_PIN);
    LED_GREEN_OFF(); // Apagar LED verde inicialmente  
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
    // Inicializacion del boton sw2
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; 
    PORTC->PCR[SW2_PIN] = PORT_PCR_MUX(1);  
    GPIOC->PDDR |= (1 << SW2_PIN); 
    GPIOC->PSOR = (1 << SW2_PIN);
}
// Asumo, que teniendo los conectores hacia abajo, el boton izquierdo es SW1 y el derecho SW2
int main(void){
    led_init();
    sw_init();

    BinaryValue bin_var = BIN_00;
    while (1) {
        
        if (SW1_PRESSED()) {
            bin_var = incrementar(bin_var);
            while (SW1_PRESSED()); // Espera hasta que se libere el botón
        }
        if (SW2_PRESSED()) {
            bin_var = intercambiar(bin_var);
            while (SW2_PRESSED()); // Espera hasta que se libere el botón
        }
        controlar_LEDs(bin_var);
        delay();
    }
    return 0;
}
