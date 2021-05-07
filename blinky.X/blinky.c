/* 
 * File:   blinky.c
 * Author: john
 *
 * Created on 11 April 2021, 21:29
 */

#include <sam.h>

/* dally --- CPU busy-loop for crude time delay */

static void dally(const int loops)
{
    volatile int dally;
    volatile int i;

    for (dally = 0; dally < loops; dally++)
    {
        for (i = 0; i < 100; i++)
        {
            __asm("nop");
        }
    }
}


/* initMCU --- set up the microcontroller in general */

static void initMCU(void)
{
    // SAMD starts up at 1MHz, using 8MHz oscillator and divide-by-8 prescaler
    
    // Change prescaler to divide-by-1
    SYSCTRL_REGS->SYSCTRL_OSC8M = (SYSCTRL_REGS->SYSCTRL_OSC8M & ~SYSCTRL_OSC8M_PRESC_Msk) | SYSCTRL_OSC8M_PRESC_0;
}


int main(void)
{
    initMCU();
    
    PORT_REGS->GROUP[1].PORT_DIRSET = PORT_PB10;    // LED pin to output
    PORT_REGS->GROUP[0].PORT_DIRSET = PORT_PA28;    // Logic analyser pin to output
    
    while (1)
    {
        PORT_REGS->GROUP[1].PORT_OUTCLR = PORT_PB10;    // LED on
        PORT_REGS->GROUP[0].PORT_OUTSET = PORT_PA28;
        dally(250);

        PORT_REGS->GROUP[1].PORT_OUTSET = PORT_PB10;    // LED off
        PORT_REGS->GROUP[0].PORT_OUTCLR = PORT_PA28;
        dally(250);
        
        PORT_REGS->GROUP[1].PORT_OUTCLR = PORT_PB10;    // LED on
        PORT_REGS->GROUP[0].PORT_OUTSET = PORT_PA28;
        dally(250);
        
        PORT_REGS->GROUP[1].PORT_OUTSET = PORT_PB10;    // LED off
        PORT_REGS->GROUP[0].PORT_OUTCLR = PORT_PA28;
        dally(1000);
    }
}

