/* 
 * File:   blinky.c
 * Author: john
 *
 * Created on 11 April 2021, 21:29
 */

#include <sam.h>

volatile uint32_t Milliseconds = 0;
volatile uint8_t Tick = 0;


/* TC3_Handler --- ISR for Timer/Counter 3 match/compare, used for 1ms ticker */

void TC3_Handler(void)
{
    Milliseconds++;
    Tick = 1;
    
    PORT_REGS->GROUP[0].PORT_OUTTGL = PORT_PA27;    // 500Hz on PA27
    
    // Acknowledge interrupt
    TC3_REGS->COUNT16.TC_INTFLAG |= TC_INTFLAG_MC0(1);
}


/* millis --- return milliseconds since reset */

uint32_t millis(void)
{
    return (Milliseconds);
}


/* t1ou --- send a byte to UART0 by polling */

void t1ou(const int ch)
{
    while ((SERCOM0_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE(1)) == 0)
        ;
    
    SERCOM0_REGS->USART_INT.SERCOM_DATA = ch;
}


/* t1ou1 --- send a byte to UART1 by polling */

void t1ou1(const int ch)
{
    while ((SERCOM1_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE(1)) == 0)
        ;
    
    SERCOM1_REGS->USART_INT.SERCOM_DATA = ch;
}


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
    
    // Power Manager setup for APB prescaler
    PM_REGS->PM_APBCSEL = PM_APBCSEL_APBCDIV_DIV1;
    
    // Select 8MHz OSC clock for GCLK1
    GCLK_REGS->GCLK_GENCTRL = GCLK_GENCTRL_IDC(1) | GCLK_GENCTRL_GENEN(1) | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSC8M_Val) | GCLK_GENCTRL_ID(1);
    
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;
    
    // Select 8MHz OSC clock for GCLK2
    GCLK_REGS->GCLK_GENCTRL = GCLK_GENCTRL_IDC(1) | GCLK_GENCTRL_GENEN(1) | GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSC8M_Val) | GCLK_GENCTRL_ID(2);
    
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;
}


/* initGPIOs --- set up the GPIO pins */

static void initGPIOs(void)
{
    PORT_REGS->GROUP[1].PORT_DIRSET = PORT_PB10;    // LED pin to output
    PORT_REGS->GROUP[0].PORT_DIRSET = PORT_PA28;    // Logic analyser pin to output
    PORT_REGS->GROUP[0].PORT_DIRSET = PORT_PA27;    // 500Hz square wave
}


/* initUARTs --- set up UART(s) and buffers, and connect to 'stdout' */

static void initUARTs(void)
{
    // Connect GCLK1 to SERCOM0
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN(1) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_ID_SERCOM0_CORE;
    
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;
    
    // Connect GCLK2 to SERCOM1
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN(1) | GCLK_CLKCTRL_GEN_GCLK2 | GCLK_CLKCTRL_ID_SERCOM1_CORE;
    
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;
    
    // Power Manager setup to supply clock to SERCOM0 and SERCOM1
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_SERCOM0(1) | PM_APBCMASK_SERCOM1(1);

    // Set up SERCOM0 as UART0
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_TXPO_PAD0 |
                                           SERCOM_USART_INT_CTRLA_RXPO_PAD1 |
                                           SERCOM_USART_INT_CTRLA_FORM_USART_FRAME_NO_PARITY |
                                           SERCOM_USART_INT_CTRLA_DORD_LSB |
                                           SERCOM_USART_INT_CTRLA_CMODE_ASYNC |
                                           SERCOM_USART_INT_CTRLA_SAMPR_16X_ARITHMETIC |
                                           SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK;
    
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_CTRLB(1))
        ;
    
    SERCOM0_REGS->USART_INT.SERCOM_CTRLB = SERCOM_USART_INT_CTRLB_CHSIZE_8_BIT |
                                           SERCOM_USART_INT_CTRLB_SBMODE_1_BIT |
                                           SERCOM_USART_INT_CTRLB_TXEN(1) |
                                           SERCOM_USART_INT_CTRLB_RXEN(1);
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_CTRLB(1))
        ;
    
    
    SERCOM0_REGS->USART_INT.SERCOM_BAUD = 64277;
    //SERCOM0_REGS->USART_INT.SERCOM_INTENSET = 0;

    PORT_REGS->GROUP[0].PORT_PMUX[4] = PORT_PMUX_PMUXE_C | PORT_PMUX_PMUXO_C;
    PORT_REGS->GROUP[0].PORT_PINCFG[8] = PORT_PINCFG_PMUXEN(1);  // TxD
    PORT_REGS->GROUP[0].PORT_PINCFG[9] = PORT_PINCFG_PMUXEN(1);  // RxD
    
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_ENABLE(1))
        ;
    
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE(1);
    
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_ENABLE(1))
        ;
    
    // Set up SERCOM1 as UART1
    SERCOM1_REGS->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_TXPO_PAD0 |
                                           SERCOM_USART_INT_CTRLA_RXPO_PAD1 |
                                           SERCOM_USART_INT_CTRLA_FORM_USART_FRAME_NO_PARITY |
                                           SERCOM_USART_INT_CTRLA_DORD_LSB |
                                           SERCOM_USART_INT_CTRLA_CMODE_ASYNC |
                                           SERCOM_USART_INT_CTRLA_SAMPR_16X_ARITHMETIC |
                                           SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK;
    
    while (SERCOM1_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_CTRLB(1))
        ;
    
    SERCOM1_REGS->USART_INT.SERCOM_CTRLB = SERCOM_USART_INT_CTRLB_CHSIZE_8_BIT |
                                           SERCOM_USART_INT_CTRLB_SBMODE_1_BIT |
                                           SERCOM_USART_INT_CTRLB_TXEN(1) |
                                           SERCOM_USART_INT_CTRLB_RXEN(1);
    while (SERCOM1_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_CTRLB(1))
        ;
    
    
    SERCOM1_REGS->USART_INT.SERCOM_BAUD = 64277;
    //SERCOM1_REGS->USART_INT.SERCOM_INTENSET = 0;

    PORT_REGS->GROUP[0].PORT_PMUX[8] = PORT_PMUX_PMUXE_C | PORT_PMUX_PMUXO_C;
    PORT_REGS->GROUP[0].PORT_PINCFG[16] = PORT_PINCFG_PMUXEN(1);  // TxD
    PORT_REGS->GROUP[0].PORT_PINCFG[17] = PORT_PINCFG_PMUXEN(1);  // RxD
    
    while (SERCOM1_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_ENABLE(1))
        ;
    
    SERCOM1_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE(1);
    
    while (SERCOM1_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_ENABLE(1))
        ;
}


/* initMillisecondTimer --- set up a timer to interrupt every millisecond */

static void initMillisecondTimer(void)
{
    // Connect GCLK1 to TC3
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN(1) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_ID_TCC2_TC3;
    
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;
    
    // Power Manager setup to supply clock to TC3
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_TC3(1);
    
    // Set up TC3 for regular 1ms interrupt
    TC3_REGS->COUNT16.TC_CTRLA = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV64 | TC_CTRLA_WAVEGEN_MFRQ;
    TC3_REGS->COUNT16.TC_CTRLC = 0;
    TC3_REGS->COUNT16.TC_CC[0] = 124;
    TC3_REGS->COUNT16.TC_INTENSET = TC_INTENSET_MC0(1);
    
    /* Set TC3 Interrupt Priority to Level 3 */
    NVIC_SetPriority(TC3_IRQn, 3);
    
    /* Enable TC3 NVIC Interrupt Line */
    NVIC_EnableIRQ(TC3_IRQn);
    
    TC3_REGS->COUNT16.TC_CTRLA |= TC_CTRLA_ENABLE(1);
    
    // Wait until TC3 is enabled
    //while(TC3->COUNT16.STATUS.bit.SYNCBUSY == 1)
    //    ;
}


int main(void)
{
    uint32_t end;
    
    initMCU();
    initGPIOs();
    initUARTs();
    initMillisecondTimer();
    
    __enable_irq();   // Enable interrupts
    
    end = millis() + 500;
    
    while (1)
    {
        if (Tick)
        {
            if (millis() >= end)
            {
                end = millis() + 500;
                PORT_REGS->GROUP[1].PORT_OUTTGL = PORT_PB10;    // LED on PB10 toggle
                PORT_REGS->GROUP[0].PORT_OUTTGL = PORT_PA28;    // Logic analyser pin toggle
                t1ou('S');
                t1ou('A');
                t1ou('M');
                t1ou('D');
                t1ou('_');
                t1ou('U');
                t1ou('0');
                t1ou(' ');
                
                t1ou1('U');
                t1ou1('1');
                t1ou1(' ');
            }
            
            Tick = 0;
        }
    }
}

