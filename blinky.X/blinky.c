/* 
 * File:   blinky.c
 * Author: john
 *
 * Created on 11 April 2021, 21:29
 */

#include <sam.h>
#include <stdio.h>

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


/* t1ou5 --- send a byte to UART5 by polling */

void t1ou5(const int ch)
{
    while ((SERCOM5_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE(1)) == 0)
        ;
    
    SERCOM5_REGS->USART_INT.SERCOM_DATA = ch;
}


/* _mon_putc --- connect UART5 to 'stdout' */

void _mon_putc(const char ch)
{
    if (ch == '\n')
    {
        t1ou5('\r');
    }
    
    t1ou5(ch);
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


/* printDeviceID --- print the Device ID bytes as read from DSU */

void printDeviceID(void)
{
   const uint32_t did = DSU_REGS->DSU_DID;
   
   const uint32_t processor = (did & DSU_DID_PROCESSOR_Msk) >> DSU_DID_PROCESSOR_Pos;
   const uint32_t family    = (did & DSU_DID_FAMILY_Msk) >> DSU_DID_FAMILY_Pos;
   const uint32_t series    = (did & DSU_DID_SERIES_Msk) >> DSU_DID_SERIES_Pos;
   const uint32_t die       = (did & DSU_DID_DIE_Msk) >> DSU_DID_DIE_Pos;
   const uint32_t revision  = (did & DSU_DID_REVISION_Msk) >> DSU_DID_REVISION_Pos;
   const uint32_t devsel    = (did & DSU_DID_DEVSEL_Msk) >> DSU_DID_DEVSEL_Pos;
   
   printf("Device ID = %08x %d %d %d %d %d %d\n", did, processor, family, series, die, revision, devsel);
}


/* printSerialNumber --- print the chip's 128-bit unique serial number */

void printSerialNumber(void)
{
   // See SAM D21/DA1 Family datasheet DS40001882F, section 10.3.3
   const uint32_t *const p0 = (const uint32_t *const)0x0080A00C;
   const uint32_t *const p1 = (const uint32_t *const)0x0080A040;
   const uint32_t *const p2 = (const uint32_t *const)0x0080A044;
   const uint32_t *const p3 = (const uint32_t *const)0x0080A048;
   
   printf("Serial Number = %08x %08x %08x %08x\n", *p0, *p1, *p2, *p3);
}


/* printResetReason --- print the cause of the chip's reset */

void printResetReason(void)
{
   printf("RCAUSE = 0x%02x ", PM_REGS->PM_RCAUSE);
   
   if (PM_REGS->PM_RCAUSE & PM_RCAUSE_SYST_Msk)
   {
      printf("SYST ");
   }
   
   if (PM_REGS->PM_RCAUSE & PM_RCAUSE_WDT_Msk)
   {
      printf("WDT ");
   }
   
   if (PM_REGS->PM_RCAUSE & PM_RCAUSE_EXT_Msk)
   {
      printf("EXT ");
   }
   
   if (PM_REGS->PM_RCAUSE & PM_RCAUSE_BOD33_Msk)
   {
      printf("BOD33 ");
   }
   
   if (PM_REGS->PM_RCAUSE & PM_RCAUSE_BOD12_Msk)
   {
      printf("BOD12 ");
   }
   
   if (PM_REGS->PM_RCAUSE & PM_RCAUSE_POR_Msk)
   {
      printf("POR ");
   }
   
   puts("");
}


/* nudgeWatchdog --- reset the watchdog counter */

void nudgeWatchdog(void)
{
    if ((WDT_REGS->WDT_STATUS & WDT_STATUS_SYNCBUSY_Msk) == 0)
    {
        WDT_REGS->WDT_CLEAR = 0xA5;
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
    
    // Select 32.768kHz OSCULP32K clock for GCLK2 to drive watchdog
    GCLK_REGS->GCLK_GENDIV = GCLK_GENDIV_DIV(4) | GCLK_GENDIV_ID(2); // Divide by 32 to give 1.024kHz
    
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;
    
    GCLK_REGS->GCLK_GENCTRL = GCLK_GENCTRL_IDC(1) | GCLK_GENCTRL_GENEN(1) |
                              GCLK_GENCTRL_DIVSEL_DIV2 |
                              GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSCULP32K_Val) | GCLK_GENCTRL_ID(2);
    
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;
}


/* initGPIOs --- set up the GPIO pins */

static void initGPIOs(void)
{
    // Power Manager setup to supply clock to PORT
    PM_REGS->PM_APBBMASK |= PM_APBBMASK_PORT(1);
    
    PORT_REGS->GROUP[1].PORT_DIRSET = PORT_PB10;    // LED pin to output
    PORT_REGS->GROUP[0].PORT_DIRSET = PORT_PA28;    // Logic analyser pin to output
    PORT_REGS->GROUP[0].PORT_DIRSET = PORT_PA27;    // 500Hz square wave
}


/* initUARTs --- set up UART(s) and buffers */

static void initUARTs(void)
{
    // Connect GCLK1 to SERCOM0
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN(1) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_ID_SERCOM0_CORE;
    
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;
    
    // Power Manager setup to supply clock to SERCOM0
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_SERCOM0(1);

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
    
    SERCOM0_REGS->USART_INT.SERCOM_BAUD = 64277;    // 9600 baud
    //SERCOM0_REGS->USART_INT.SERCOM_INTENSET = 0;

    PORT_REGS->GROUP[0].PORT_PMUX[4] = PORT_PMUX_PMUXE_C | PORT_PMUX_PMUXO_C;
    PORT_REGS->GROUP[0].PORT_PINCFG[8] = PORT_PINCFG_PMUXEN(1);  // PA8 TxD
    PORT_REGS->GROUP[0].PORT_PINCFG[9] = PORT_PINCFG_PMUXEN(1);  // PA9 RxD
    
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_ENABLE(1))
        ;
    
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE(1);
    
    while (SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_ENABLE(1))
        ;
    
    // Connect GCLK1 to SERCOM5
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN(1) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_ID_SERCOM5_CORE;
    
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;
    
    // Power Manager setup to supply clock to SERCOM5
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_SERCOM5(1);

    // Set up SERCOM5 as UART5
    SERCOM5_REGS->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_TXPO_PAD0 |
                                           SERCOM_USART_INT_CTRLA_RXPO_PAD2 |
                                           SERCOM_USART_INT_CTRLA_FORM_USART_FRAME_NO_PARITY |
                                           SERCOM_USART_INT_CTRLA_DORD_LSB |
                                           SERCOM_USART_INT_CTRLA_CMODE_ASYNC |
                                           SERCOM_USART_INT_CTRLA_SAMPR_16X_ARITHMETIC |
                                           SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK;
    
    while (SERCOM5_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_CTRLB(1))
        ;
    
    SERCOM5_REGS->USART_INT.SERCOM_CTRLB = SERCOM_USART_INT_CTRLB_CHSIZE_8_BIT |
                                           SERCOM_USART_INT_CTRLB_SBMODE_1_BIT |
                                           SERCOM_USART_INT_CTRLB_TXEN(1) |
                                           SERCOM_USART_INT_CTRLB_RXEN(1);
    while (SERCOM5_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_CTRLB(1))
        ;
    
    SERCOM5_REGS->USART_INT.SERCOM_BAUD = 64277;    // 9600 baud
    //SERCOM5_REGS->USART_INT.SERCOM_INTENSET = 0;

    PORT_REGS->GROUP[0].PORT_PMUX[11] = PORT_PMUX_PMUXE_D;
    PORT_REGS->GROUP[1].PORT_PMUX[11] = PORT_PMUX_PMUXE_D;
    PORT_REGS->GROUP[0].PORT_PINCFG[22] = PORT_PINCFG_PMUXEN(1);  // PA22 TxD
    PORT_REGS->GROUP[1].PORT_PINCFG[22] = PORT_PINCFG_PMUXEN(1);  // PB22 RxD
    
    while (SERCOM5_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_ENABLE(1))
        ;
    
    SERCOM5_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE(1);
    
    while (SERCOM5_REGS->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_ENABLE(1))
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


/* initDAC --- set up the 10-bit Digital-to-Analog Converter */

static void initDAC(void)
{
    // Connect GCLK1 to DAC
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN(1) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_ID_DAC;
    
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;
    
    // Power Manager setup to supply clock to DAC
    PM_REGS->PM_APBCMASK |= PM_APBCMASK_DAC(1);
    
    // Set up I/O pins
    PORT_REGS->GROUP[0].PORT_PMUX[1] = PORT_PMUX_PMUXE_B;
    PORT_REGS->GROUP[0].PORT_PINCFG[2] = PORT_PINCFG_PMUXEN(1);   // Vout on PA2
    
    // Set up DAC registers
    DAC_REGS->DAC_CTRLA = 0;
    DAC_REGS->DAC_CTRLB = DAC_CTRLB_REFSEL(1) | DAC_CTRLB_EOEN(1);
    DAC_REGS->DAC_DATA  = 0;
    
    DAC_REGS->DAC_CTRLA |= DAC_CTRLA_ENABLE(1);
}


/* initWatchdog --- set up the watchdog timer */

static void initWatchdog(void)
{
    // Connect GCLK2 to WDT
    GCLK_REGS->GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN(1) | GCLK_CLKCTRL_GEN_GCLK2 | GCLK_CLKCTRL_ID_WDT;
    
    while (GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk)
        ;
    
    // Power Manager setup to supply clock to WDT
    PM_REGS->PM_APBAMASK |= PM_APBAMASK_WDT(1);
    
    WDT_REGS->WDT_CTRL = WDT_CTRL_WEN(0) | WDT_CTRL_ENABLE(0);
    WDT_REGS->WDT_CONFIG = WDT_CONFIG_PER_16K;      // Period 16384 gives 16 seconds
    
    while (WDT_REGS->WDT_STATUS & WDT_STATUS_SYNCBUSY_Msk)
        ;
    
    WDT_REGS->WDT_CTRL |= WDT_CTRL_ENABLE(1);  // Start the watchdog
    
    while (WDT_REGS->WDT_STATUS & WDT_STATUS_SYNCBUSY_Msk)
        ;
}


int main(void)
{
    uint32_t end;
    uint16_t ramp = 0;
    
    initMCU();
    initGPIOs();
    initUARTs();
    initMillisecondTimer();
    initDAC();
    initWatchdog();
    
    __enable_irq();   // Enable interrupts
    
    printf("\nHello from the SAM%c%d%c%d%c\n", 'D', 21, 'G', 17, 'D');
    
    printResetReason();
    printDeviceID();
    printSerialNumber();
   
    end = millis() + 500;
    
    while (1)
    {
        if (Tick)
        {
            DAC_REGS->DAC_DATA = ramp++;
            
            if (millis() >= end)
            {
                end = millis() + 500;
                PORT_REGS->GROUP[1].PORT_OUTTGL = PORT_PB10;    // LED on PB10 toggle
                PORT_REGS->GROUP[0].PORT_OUTTGL = PORT_PA28;    // Logic analyser pin toggle
                t1ou5('S');
                t1ou5('A');
                t1ou5('M');
                t1ou5('D');
                t1ou5('_');
                t1ou5('U');
                t1ou5('5');
                t1ou5(' ');
                
                t1ou('U');
                t1ou('0');
                t1ou(' ');
            }
            
            nudgeWatchdog();
            Tick = 0;
        }
    }
}

