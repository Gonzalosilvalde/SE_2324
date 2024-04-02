#include "MKL46Z4.h"
#include "lcd.h"
#include "division_asm.h"


//funciones
void irclk_ini()
{
    MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
    MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}






int main(void)
{
    irclk_ini(); // Enable internal ref clk to use by LCD

    lcd_ini();
    
    while (1) {
            LCD->AR |= LCD_AR_BLINK(1);
            LCD->AR |= LCD_AR_BRATE(0xBB);
            lcd_display_dec(6565);

        
    }

    return 0;
}
