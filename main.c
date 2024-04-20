#include "MKL46Z4.h"
#include "fsl_port.h"
#include "board.h"


#define APP_WAKEUP_BUTTON_PORT BOARD_SW1_PORT
#define APP_WAKEUP_BUTTON_GPIO_PIN BOARD_SW1_GPIO_PIN
#define APP_WAKEUP_BUTTON_PORT2 BOARD_SW3_PORT
#define APP_WAKEUP_BUTTON_GPIO_PIN2 BOARD_SW3_GPIO_PIN
#define APP_WAKEUP_BUTTON_IRQ_TYPE kPORT_InterruptFallingEdge

// Definicion de constantes minicom -D /dev/tty(ver cual es, ya que aqui hai varios) darle al reset y deberia enviar un gello world por pantalla
#define LED_GREEN_PIN 5
#define LED_RED_PIN 29
#define SW1_PIN 3
#define SW2_PIN 12


// MACROS para ahorrar espacion en ejecutable y reducir sobrecarga
// MACROS LED
//letra extra añadida por culpa de board.h
#define LED_GREEN_ONN()   (GPIOD->PCOR = (1 << LED_GREEN_PIN)) // Encender LED verde
#define LED_GREEN_OFFF()  (GPIOD->PSOR = (1 << LED_GREEN_PIN)) // Apagar LED verde
#define LED_RED_ONN()     (GPIOE->PCOR = (1 << LED_RED_PIN))   // Encender LED rojo
#define LED_RED_OFFF()    (GPIOE->PSOR = (1 << LED_RED_PIN))   // Apagar LED rojo

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

BinaryValue bin_var = BIN_00;//Como variable global funciona mejor

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
void controlar_LEDs() {
    // LED Rojo
    if (bin_var & BIN_01) {
        LED_RED_ONN();  // Enciende el LED rojo
    } else {
        LED_RED_OFFF();  // Apaga el LED rojo
    }
    // LED Verde
    if (bin_var & BIN_10) {
        LED_GREEN_ONN();  // Enciende el LED verde
    } else {
        LED_GREEN_OFFF();  // Apaga el LED verde
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
    PORTC->PCR[SW1_PIN] |= PORT_PCR_IRQC(0xA); // IRQ en el flanco de bajada
    // Inicializacion del boton sw2
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; 
    PORTC->PCR[SW2_PIN] = PORT_PCR_MUX(1);  
    GPIOC->PDDR |= (1 << SW2_PIN); 
    GPIOC->PSOR = (1 << SW2_PIN);
    PORTC->PCR[SW2_PIN] |= PORT_PCR_IRQC(0xA); // IRQ en el flanco de bajada
	NVIC_SetPriority(31, 0);	// Bajar prioridad
	NVIC_EnableIRQ(31);			// Activa la interrupcion

}


void PORTDIntHandler(void) {
	PORTC->ISFR = 0xFFFFFFFF;	// Clear IRQ
    
    if (SW1_PRESSED()) {
        PORTC->PCR[SW1_PIN] |= PORT_PCR_ISF_MASK;
        bin_var = incrementar(bin_var);
        while (SW1_PRESSED()); // Espera hasta que se libere el botón
    }
    if (SW2_PRESSED()) {
        PORTC->PCR[SW2_PIN] |= PORT_PCR_ISF_MASK;

        bin_var = intercambiar(bin_var);
        while (SW2_PRESSED()); // Espera hasta que se libere el botón
    }
    controlar_LEDs();
}

// Asumo, que teniendo los conectores hacia abajo, el boton izquierdo es SW1 y el derecho SW2
int main(void){
    led_init();
    sw_init();

    PORT_SetPinInterruptConfig(APP_WAKEUP_BUTTON_PORT, APP_WAKEUP_BUTTON_GPIO_PIN, APP_WAKEUP_BUTTON_IRQ_TYPE);
    PORT_SetPinInterruptConfig(APP_WAKEUP_BUTTON_PORT2, APP_WAKEUP_BUTTON_GPIO_PIN2, APP_WAKEUP_BUTTON_IRQ_TYPE);


    while (1) {

    }


    return 0;
}