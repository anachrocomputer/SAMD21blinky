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
    
    // Set up one wait state for 48MHz
    NVMCTRL_REGS->NVMCTRL_CTRLB = (NVMCTRL_REGS->NVMCTRL_CTRLB & ~NVMCTRL_CTRLB_RWS_Msk) | (1 << NVMCTRL_CTRLB_RWS_Pos);
    
    // Work around erratum
    while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLRDY_Msk) == 0)
        ;
    
    SYSCTRL_REGS->SYSCTRL_DFLLCTRL = SYSCTRL_DFLLCTRL_ENABLE(1);
    
    while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLRDY_Msk) == 0)
        ;
    
    // Read calibration
    uint32_t coarse = ((*(uint32_t*)(OTP4_ADDR + 4)) & 0xFC000000) >> 26;
    uint32_t fine = (*(uint32_t*)(OTP4_ADDR + 8)) & 0x3FF;
    
    // Write calibration
    SYSCTRL_REGS->SYSCTRL_DFLLVAL = SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine); 
    
    while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & SYSCTRL_PCLKSR_DFLLRDY_Msk) == 0)
        ;
    
    // Select 48MHz DFLL clock
    GCLK_REGS->GCLK_GENCTRL = GCLK_GENCTRL_IDC(1) | GCLK_GENCTRL_GENEN(1) | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL48M_Val) | GCLK_GENCTRL_ID(0);
    
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;
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

