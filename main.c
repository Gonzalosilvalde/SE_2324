#include "fsl_debug_console.h"

#include "fsl_pit.h"
#include "MKL46Z4.h"
#include "lcd.h"
#include "pin_mux.h"
#include "clock_config.h"




//macros/constantes relacionadas con los leds
#define LED_GREEN_PIN 5
#define LED_RED_PIN 29
#define LED_GREEN_ON()   (GPIOD->PCOR = (1 << LED_GREEN_PIN))  // Encender LED verde
#define LED_GREEN_OFF()  (GPIOD->PSOR = (1 << LED_GREEN_PIN)) // Apagar LED verde
#define LED_RED_ON()     (GPIOE->PCOR = (1 << LED_RED_PIN))  // Encender LED rojo
#define LED_RED_OFF()    (GPIOE->PSOR = (1 << LED_RED_PIN)) // Apagar LED rojo
//funcion para intercambiar estado leds
#define LED_TOGGLE() \
do { \
    static uint8_t toggle_count = 0; \
    if (toggle_count % 2 == 0) { \
        LED_GREEN_ON(); \
        LED_RED_OFF(); \
    } else { \
        LED_GREEN_OFF(); \
        LED_RED_ON(); \
    } \
    toggle_count++; \
} while(0)


//macros/constantes relacionadas con los botones
#define SW1_PIN 3
#define SW2_PIN 12
#define SW1_PRESSED()    ((GPIOC->PDIR & (1 << SW1_PIN)) == 0) // Verificar si SW1 está presionado
#define SW2_PRESSED()    ((GPIOC->PDIR & (1 << SW2_PIN)) == 0) // Verificar si SW2 está presionado

//constantes/macros relacionadas con los segundos minutos
#define MINUTES_MAX 59

void INC_COUNTER(int *counter) {
    asm volatile (
        "ldr r1, [%0]\n"          // Carga el valor almacenado en la dirección apuntada por counter
        "add r1, r1, #1\n"       // Incrementa el valor de r1 en 1
        "cmp r1, %1\n"          // Compara el valor de r1 con MINUTES_MAX
        "ble .done\n"          // Si el valor de r1 es menor o igual a MINUTES_MAX, salta a .done
        "movs r1, #0\n"       // Si el valor de r1 excede MINUTES_MAX, lo reinicia a 0
        ".done:\n"           // Etiqueta para finalizar el proceso
        "str r1, [%0]\n"    // Almacena el valor final de r1 en la dirección apuntada por counter
        :
        : "r" (counter), "i" (MINUTES_MAX)
        : "r1"
    );
}


void DEC_COUNTER(int *counter) {
    asm volatile (
        "ldr r1, [%0]\n"                   // Carga el valor
        "cmp r1, #0\n"                    // Compara con cero
        "ble .reset\n"                   // Si es menor o igual, lo establece al valor de MINUTES_MAX (59)
        "sub r1, r1, #1\n"              // Decrementa el valor en 1
        "b .mask\n"                    // Salta a la etiqueta .mask
        ".reset:\n"                   // Etiqueta para el caso de reinicio
        "mov r1, %1\n"               // Establece el valor de r1 a MINUTES_MAX
        ".mask:\n"                  // Etiqueta para la máscara
        "mov r2, #0x3F\n"          // Carga 0x3F en r2
        "and r1, r1, r2\n"        // Aplica una máscara a r1 de 6 bits
        "str r1, [%0]\n"         // Guarda el valor actualizado
        :
        : "r" (counter), "i" (MINUTES_MAX)
        : "r1", "r2"
    );
}
/*
//suma 1 al counter, cuando llega a 59, regresa a 0
#define INC_COUNTER(counter) do { \
                                (counter) = ((counter) + 1) % (MINUTES_MAX + 1); \
                            } while(0)
//resta 1 al counter, cuando llega a 0, regresa a 59
#define DEC_COUNTER(counter) do { \
                                if ((counter) == 0) (counter) = MINUTES_MAX + 1; \
                                (counter) = ((counter) - 1) & 0x3F; \
                            } while(0)
*/


//configuracion del pit
#define PIT_IRQ_HANDLER PITIntHandler
#define PIT_IRQ_ID PIT_IRQn

#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)


pit_config_t pitConfig;

//iniciar variables globales
int counter_s = 0;        //contador de segundos
int counter_m = 0;       //contador de minutos
volatile uint64_t old_counter_s = 0;  //guardar valor viejo de contador segundos
volatile uint64_t old_counter_m = 0; //guardar valor viejo de contador minutos
volatile uint64_t estado = 0;       //estado en el ciclo
volatile uint64_t parada = 0;      //condicion para ver si esta pausado el contador




  //funcion de delay modificada:
 //2000000 para que la suma/resta de segundos/minutos se vea fluida pero no muy rapida 
//4000000 para que al pulsar el boton sw2/sw1 para cambiar de estado, de levantar los dedos para no modificar sin querer segundos/minutos
void delay(int tiempo) 
{
    volatile int i;

    for (i = 0; i < tiempo; i++);
}
#define DELAY_SHORT() (delay(2000000))
#define DELAY_LONG() (delay(4000000))

void irclk_ini() 
{
    MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
    MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}


void leds_init() 
{
    SIM->COPC = 0;
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
    PORTD->PCR[5] = PORT_PCR_MUX(1); // LED_GREEN
    PORTE->PCR[29] = PORT_PCR_MUX(1); // LED_RED
    GPIOD->PDDR |= (1 << 5);
    GPIOE->PDDR |= (1 << 29);
    // Turn off LED_GREEN
    GPIOD->PSOR = (1 << 5);
    
}

//inicializar botones
void sw_init() 
{
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

//cuando se vaya a iniciar el contador, se llama a esta funcion
void confPit(void)
{
    NVIC_DisableIRQ(PORTC_PORTD_IRQn); // Desactivar las interrupciones de los botones para no tocar nada durante la cuenta atras
    
    //iniciar todo lo relacionado con el PIT
    PIT_GetDefaultConfig(&pitConfig);

    PIT_Init(PIT, &pitConfig);

    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(1000000U, PIT_SOURCE_CLOCK));

    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
    PIT_SetTimerChainMode(PIT, kPIT_Chnl_0,true);

    //iniciar interrupciones del PIT
    EnableIRQ(PIT_IRQ_ID);


    PIT_StartTimer(PIT, kPIT_Chnl_0);
}

//interrupciones de los botones
void PORTDIntHandler(void) {
	PORTC->ISFR = 0xFFFFFFFF;	// Clear IRQ
    if(estado < 2)
    {
        //Mientras se este pulsando cualquiera de los botones, se mantiene en el bucle, sumando o restando segundos/minutos
        while(SW1_PRESSED() || SW2_PRESSED()) 
        {
            if(SW2_PRESSED() && SW1_PRESSED() && estado == 0)
            {
                LED_TOGGLE();
                estado = 1;
                DELAY_LONG();
                break;
            }
            if(SW1_PRESSED() && SW2_PRESSED() && estado == 1)
            {
                old_counter_s = counter_s;          // variable para guardar el valor de los segundos para el futuro
                old_counter_m = counter_m;         // variable para guardar el valor de los minutos para el futuro
    
                confPit();
                estado = 2;
                break;
            }
            
            DELAY_SHORT();

            if (estado == 0 && SW1_PRESSED()) {           //si se esta en el estado 0 y se pulsa el sw1, suma 1 a los segundos
                INC_COUNTER(&counter_s);
            } else if (estado == 1 && SW1_PRESSED()){    //si se esta en el estado 1 y se pulsa el sw1, suma 1 a los minutos
                INC_COUNTER(&counter_m);
            }else if (estado == 0 && SW2_PRESSED()){    //si se esta en el estado 0 y se pulsa el sw2, resta 1 a los segundos
                DEC_COUNTER(&counter_s);
            }else if (estado == 1 && SW2_PRESSED()){   //si se esta en el estado 1 y se pulsa el sw2, resta 1 a los minutos
                DEC_COUNTER(&counter_m);
            }
            lcd_display_time(counter_m, counter_s);  //muestra los tiempos modificados por el lcd
        }
    }
    else if (SW2_PRESSED() && estado == 2)//reactiva el PIT despues de una parada
    {
        confPit();
    }
    else if(SW1_PRESSED() && estado == 3) //si se esta en el estado 2 y se pulsa sw1, recupera los valores iniciales 
                                         //de los tiempos y reinicia la cuenta atras
    {
        counter_m = old_counter_m;
        counter_s = old_counter_s;
        estado = 2;                      //se vuelve al estado 2 para evitar que al pausar en la "segunda vuelta" se vuelva al estado 0
        confPit();
    }
    if(SW2_PRESSED() && estado == 3)     //si se esta en el estado 2 y se pulsa sw1,
                                        // vuelves a tener la opcion de modificar los tiempos
    {
        estado = 0;
    }
    
}

//interrupciones del PIT
void PIT_IRQ_HANDLER(void)
{
    
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);// Clear IRQ
    
    if(!parada)             //si no se para el contador
    {
        LED_TOGGLE();        //se enciende/apaga los leds
        if (counter_s == 0) //si el contador de segundos llega a 0
        {
            DEC_COUNTER(&counter_s); //se pasan los segundos a 59
            DEC_COUNTER(&counter_m);//se resta 1 a los minuts
        }
        else                //si el contador de segundos tiene un valor mayor a 0
        {
            DEC_COUNTER(&counter_s); //resta 1 segundo

        }
    }
    if(SW1_PRESSED()){ //si se pulsa sw1, se desactivan las interrupciones del PIT y se activan las de sw1
        NVIC_EnableIRQ(PORTC_PORTD_IRQn);
        PIT_DisableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
           
    }

    lcd_display_time(counter_m, counter_s);
    if(counter_m == 0 && counter_s==0) //Si se llega a 0 en ambos contadores
    {
        NVIC_EnableIRQ(PORTC_PORTD_IRQn);                                     //activa las interrupciones de los botones para elegir si volver a 
                                                                             //iniciar desde el counter iniciar o si se quiere poner tiempos nuevos
        PIT_DisableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable); //se desactivan las interrupciones de PIT
        estado = 3;                     //se pasa al estado 3

    }

}



int main(void)
{
    //se inicia el lcd, PIT,  botones y led
    irclk_ini();
    lcd_ini();
    sw_init();
    leds_init();

    //tiempo inicial 00:00
    lcd_display_time(00,00);
    while (true)
    {

    }
}