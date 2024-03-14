#include "MKL46Z4.h"
#include "lcd.h"

//constantes
#define LED_GREEN_PIN 5
#define LED_RED_PIN 29
#define SW1_PIN 3
#define SW2_PIN 12



//structura para guardar los hits y los misses
typedef struct {
    uint8_t hits;
    uint8_t misses;
} GameStats;
// Declaración de variables globales
volatile uint8_t LedS = 0; // Variable para el LED verde (0 = apagado, 1 = encendido)

uint8_t i = 0;
volatile unsigned int sequence = 0x32B14D98;

GameStats gameStats = {0};

// Definiciones de macros con cambio de variables globales
#define LED_GREEN_ON_RED_OFF()  LedS = 1; GPIOD->PCOR = (1 << LED_GREEN_PIN); GPIOE->PSOR = (1 << LED_RED_PIN); // Encender LED verde y establecer LedS a 1 
#define LED_GREEN_OFF_RED_ON()  LedS = 0; GPIOD->PSOR = (1 << LED_GREEN_PIN); GPIOE->PCOR = (1 << LED_RED_PIN); // Apagar LED verde y establecer LedS a 0 

// MACROS estado botones
#define SW1_PRESSED()    ((GPIOC->PDIR & (1 << SW1_PIN)) == 0) // Verificar si SW1 está presionado
#define SW2_PRESSED()    ((GPIOC->PDIR & (1 << SW2_PIN)) == 0) // Verificar si SW2 está presionado

//MACRO Funcion interna para comprobar estado del LedS
#define IS_HIT() (SW1_PRESSED() && LedS) || (SW2_PRESSED() && !LedS)

//funciones
void irclk_ini()
{
    MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
    MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}

// Led's
// Funcion para inicializar LEDs modificada para iniciarlos a la vez
void led_init() {
    SIM->COPC = 0; //watchdog
    // Inicialización del LED verde
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    PORTD->PCR[LED_GREEN_PIN] = PORT_PCR_MUX(1);
    GPIOD->PDDR |= (1 << LED_GREEN_PIN);
    // Inicialización del LED rojo
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    PORTE->PCR[LED_RED_PIN] = PORT_PCR_MUX(1);
    GPIOE->PDDR |= (1 << LED_RED_PIN);
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
    PORTC->PCR[SW2_PIN] = PORT_PCR_MUX(1);  
    GPIOC->PDDR |= (1 << SW2_PIN); 
    GPIOC->PSOR = (1 << SW2_PIN);
    PORTC->PCR[SW2_PIN] |= PORT_PCR_IRQC(0xA); // IRQ en el flanco de bajada
	NVIC_SetPriority(31, 0);	// Bajar prioridad
	NVIC_EnableIRQ(31);			// Activa la interrupcion

}

void secuencia (void){
    if (sequence & (1 << i)) { // Comprueba el bit en la posición i
        LED_GREEN_ON_RED_OFF();     
    } else {
        LED_GREEN_OFF_RED_ON();
    }
}

void PORTDIntHandler(void) {
	PORTC->ISFR = 0xFFFFFFFF;	// Clear IRQ
    if (IS_HIT()) {
        ++gameStats.hits;
    } else {
        ++gameStats.misses;
    }
    secuencia();

    lcd_display_time(gameStats.hits, gameStats.misses);
    i++;
    if(i>31){
        NVIC_DisableIRQ(31);//descativar interrupciones
    }
    
}

int main(void)
{
    led_init();
    sw_init();
    irclk_ini(); // Enable internal ref clk to use by LCD

    lcd_ini();
    lcd_display_time(00, 00);

    secuencia();
    
    while (1) {
        if(!NVIC_GetEnableIRQ(31)){//si las interrupciones estan desactivadas, que pestanexe
            LCD->AR |= LCD_AR_BLINK(1);
            LCD->AR |= LCD_AR_BRATE(0xBB);
            lcd_display_time(gameStats.hits,gameStats.misses);
        }
    }

    return 0;
}
