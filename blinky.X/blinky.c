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

int main(void)
{
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

