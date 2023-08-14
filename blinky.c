/* blinky --- blink an LED on the SAMD21 ARM Cortex M0+     2023-08-14 */

#include <sam.h>

int main(void)
{
   volatile int dally;
   
   PORT->Group[1].DIRSET.reg = PORT_PB10;    // LED pin to output
   
   while (1) {
      PORT->Group[1].OUTSET.reg = PORT_PB10;    // LED on PB10 on
      
      for (dally = 0; dally < 800000; dally++)
         ;
      
      PORT->Group[1].OUTCLR.reg = PORT_PB10;    // LED on PB10 off
      
      for (dally = 0; dally < 800000; dally++)
         ;
   }
}
