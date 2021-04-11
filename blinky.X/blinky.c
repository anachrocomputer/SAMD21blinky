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
    PORT_REGS->GROUP[1].PORT_DIRSET = PORT_PB10;
    
    while (1)
    {
        PORT_REGS->GROUP[1].PORT_OUTCLR = PORT_PB10;
        dally(250);

        PORT_REGS->GROUP[1].PORT_OUTSET = PORT_PB10;
        dally(250);
        
        PORT_REGS->GROUP[1].PORT_OUTCLR = PORT_PB10;
        dally(250);
        
        PORT_REGS->GROUP[1].PORT_OUTSET = PORT_PB10;
        dally(1000);
    }
}

