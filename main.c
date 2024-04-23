#include "MKL46Z4.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lcd.h"
#include "semphr.h"

//LA PRIORIDAD DEL LCD DEBE DE SER LA MAYOR PARA QUE PUEDA MOSTRAR EL VALOR DE LA CUENTA
#define P_PRODUCTOR 1   //Prioridad del productor
#define P_CONSUMIDOR 1 //Prioridad del consumidor
#define P_LCD 4       //Prioridad del LCD
//macros/constantes relacionadas con los botones
#define SW1_PIN 3
#define SW2_PIN 12
#define SW1_PRESSED()    ((GPIOC->PDIR & (1 << SW1_PIN)) == 0)  // Verificar si SW1 está presionado
#define SW2_PRESSED()    ((GPIOC->PDIR & (1 << SW2_PIN)) == 0) // Verificar si SW2 está presionado

//macro para escalar el valor de la cuenta
#define LIMIT(x, min, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))


//macros/constantes relacionadas con los leds
#define LED_GREEN_PIN 5
#define LED_RED_PIN 29
#define LED_GREEN_ON()   (GPIOD->PCOR = (1 << LED_GREEN_PIN))  // Encender LED verde
#define LED_GREEN_OFF()  (GPIOD->PSOR = (1 << LED_GREEN_PIN)) // Apagar LED verde
#define LED_RED_ON()     (GPIOE->PCOR = (1 << LED_RED_PIN))  // Encender LED rojo
#define LED_RED_OFF()    (GPIOE->PSOR = (1 << LED_RED_PIN)) // Apagar LED rojo

void turnOnGreenLed() 
{
    LED_GREEN_ON();
    LED_RED_OFF();
}

void turnOnRedLed() 
{
    LED_GREEN_OFF();
    LED_RED_ON();
}

QueueHandle_t queueHandle;
static int productores = 0;
static int consumidores = 0;
static int prod_plus_com = 0;
TaskHandle_t tLCD = NULL;
TaskHandle_t tProd = NULL;
TaskHandle_t tCons = NULL;
SemaphoreHandle_t countMutex;

void irclk_ini()
{
    MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
    MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
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

//inicializar leds
void leds_init() 
{
    SIM->COPC = 0;
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
    PORTD->PCR[5] = PORT_PCR_MUX(1); // LED_GREEN
    PORTE->PCR[29] = PORT_PCR_MUX(1); // LED_RED
    GPIOD->PDDR |= (1 << 5);
    GPIOE->PDDR |= (1 << 29);
    
    
}

// Función para calcular el producto de productores por 10 y sumarle el número de consumidores
void calculate_prod_plus_com() 
{
    asm volatile(
        "ldr r0, %0\n"              //productores
        "ldr r1, %1\n"             //consumidores
        "mov r2, #10\n"           //r2 = 10
        "mul r0, r0, r2\n"       //r0 = productores * 10
        "add r0, r0, r1\n"      //r0 = productores * 10 + consumidores
        "str r0, %2\n"         //prod_plus_com = productores * 10 + consumidores
        :
        : "m" (productores), "m" (consumidores), "m" (prod_plus_com)
        : "r0", "r1", "r2"
    );
}

// Función para incrementar una variable en 1
void sumar(int* variable)
{
    asm (
        "ldr r0, %0\n"            // Cargar el valor de la variable en r0
        "add r0, r0, #1\n"       // Sumar 1 a r0
        "cmp r0, #5\n"          // Comparar r0 con 5
        "bne 1f\n"             // Si no es igual, saltar a la etiqueta 1
        "mov r0, #0\n"        // Establecer r0 en 0
        "1:\n"               // Etiqueta 1
        "str r0, %0\n"      // Almacenar el valor de r0 de nuevo en la variable
        : "+m" (*variable) // Salida: la variable es modificada
        :                 // No hay operandos de entrada
        : "r0"           // Registro afectado: r0 es modificado
    );
    calculate_prod_plus_com();
}

void PORTDIntHandler(void) 
{
    PORTC->ISFR = 0xFFFFFFFF;    // Clear IRQ
    if(SW1_PRESSED())
    {
        sumar(&consumidores); // Incrementar el número de consumidores

    }
    if(SW2_PRESSED())
    {
        sumar(&productores); // Incrementar el número de productores
    }

}

void taskConsumidor(void *pvParameters)
{
    int contador = 0;//inicializa el contador
    for (;;) 
    {//bucle infinito
        if( xSemaphoreTake( countMutex, ( TickType_t ) 110 ) == pdTRUE )    //se coge el mutex
        { 
            turnOnGreenLed();                                             //se enciende el led verde
            xQueueReceive(queueHandle, &contador, 0);                    //se recibe el valor de la cola

            if(contador>0)
            {                                        //si el contador es mayor que 0
                contador-=consumidores;             //se decrementa el contador
            }                                   
            xQueueSend(queueHandle, &contador, portMAX_DELAY);//se envia el contador a la cola
            vTaskDelay(100 / portTICK_RATE_MS);              //se pone un delay para que los otros procesos puedan coger el mutex
            xSemaphoreGive(countMutex);                     //se da acceso al mutex
        }
    }
}



void taskProductor(void *pvParameters)
{
    int contador = 0; //inicializa el contador


    for (;;) 
    {
        if( xSemaphoreTake( countMutex, ( TickType_t ) 110 ) == pdTRUE ) //se coge el mutex
        {
            turnOnRedLed();                                            //se enciende el led rojo
            xQueueReceive(queueHandle, &contador, 0);                 //se recibe el valor de la cola

            if(contador<99)              //si el contador es menor que 99
            { 
                contador+=productores; //se incrementa el contador
            }

            xQueueSend( queueHandle,  &contador, portMAX_DELAY);//se envia el contador a la cola
            vTaskDelay(100 / portTICK_RATE_MS);                //se pone un delay para que los otros procesos puedan coger el mutex
            xSemaphoreGive(countMutex);                       //se da acceso al mutex
        }
    }
}



void taskLCD(void *pvParameters)
{
    int contador = 0;
    int limited_contador;
    for (;;) 
    {

        if( xSemaphoreTake( countMutex, ( TickType_t ) 10 ) == pdTRUE )  //se coge el mutex
        {                                                               
            xQueueReceive(queueHandle, &contador, ( TickType_t ) 10);  //se recibe el valor de la cola
            
            limited_contador = LIMIT(contador, 0, 99);               //se limita el contador entre 0 y 99
            lcd_display_time(prod_plus_com, limited_contador);      //se muestra el valor en el lcd


            xSemaphoreGive(countMutex);                           //se da acceso al mutex
        }
            xQueueSend( queueHandle,  &contador, portMAX_DELAY);//se envia el contador a la cola

        //le ponemmos delay para que puedan coger el mutex los otros procesos
        vTaskDelay(100 / portTICK_RATE_MS);


    }
}




int main(void)
{
    irclk_ini();
    lcd_ini();
    leds_init();
    sw_init();
    
    queueHandle = xQueueCreate(1, sizeof(int)); // Se crea la cola con un solo elemento
    countMutex = xSemaphoreCreateMutex();      // Se crea el mutex

    //Productor
    xTaskCreate(taskProductor, "TaskProductor",
                configMINIMAL_STACK_SIZE, (int *)NULL, P_PRODUCTOR, &tProd);


    //Consumidor
     xTaskCreate(taskConsumidor, "TaskConsumidor",
                        configMINIMAL_STACK_SIZE, (int *)NULL, P_CONSUMIDOR, &tCons);


    //muestra el valor por el lcd
    xTaskCreate(taskLCD, "TaskLCD",
                configMINIMAL_STACK_SIZE, (int *)NULL, P_LCD, &tLCD);

    /* start the scheduler */
    vTaskStartScheduler();


    

	return 0;
}

