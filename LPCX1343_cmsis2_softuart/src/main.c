#include "LPC13xx.h"
#include "gpio.h"
#include "uart.h"
#include "logger.h"
#include "timer16.h"
#include "timer32.h"
#include "ext_int.h"
#include "rdm630.h"

#include <cr_section_macros.h>
#include <NXP/crp.h>

// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

// LPCXpresso processor card LED
#define LED_PORT 0		// Port for led
#define LED_BIT 7		// Bit on port for led
#define LED_ON 1		// Level to set port to turn on led
#define LED_OFF 0		// Level to set port to turn off led

unsigned int LEDvalue = LED_OFF;

extern volatile uint32_t UARTStatus;
extern volatile uint8_t UARTTxEmpty;
extern volatile uint8_t UARTBuffer[BUFSIZE];
extern volatile uint32_t UARTCount;

volatile uint32_t msTicks;                          /* counts 10ms timeTicks */


/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;                        /* increment counter necessary in Delay() */
}


void init_timers() {
	// enabled clk for timer16_0
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);
	// enabled clk for timer16_1
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<8);
	// enabled clk for timer32_0
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<9);
	// enabled clk for timer32_1
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<10);
}





/*****************************************************************************
**   Main Function  main()
******************************************************************************/
int main (void) {

   /* Setup SysTick Timer for 10 msec interrupts  */
   if (SysTick_Config(SystemCoreClock / 100)) {
		while (1); /* Capture error */
   }

   if (!(SysTick->CTRL & SysTick_CTRL_CLKSOURCE_Msk)) {
        /* When external reference clock is used(CLKSOURCE in
		Systick Control and register bit 2 is set to 0), the
		SYSTICKCLKDIV must be a non-zero value and 2.5 times
		faster than the reference clock.
		When core clock, or system AHB clock, is used(CLKSOURCE
		in Systick Control and register bit 2 is set to 1), the
		SYSTICKCLKDIV has no effect to the SYSTICK frequency. See
		more on Systick clock and status register in Cortex-M3
		technical Reference Manual. */
		LPC_SYSCON->SYSTICKCLKDIV = 0x08;
	}


   GPIOInit();

   init_timers();



   // The LED on Xpresso
   /* Set port 0_7 to output */
   GPIOSetDir( LED_PORT, LED_BIT, 1 );
   GPIOSetValue( 0, 7, LEDvalue );



   // UART
   UARTInit(115200);
   // Enable the UART Interrupt
   NVIC_EnableIRQ(UART_IRQn);
   LPC_UART->IER = IER_RBR | IER_RLS;

   logger_setEnabled(1);
   logger_logStringln("/O:entering main loop..."); // send online message (means i'm online)


   rdm630_init();
   rdm630_reset();


   while (1) {

       /* process logger */
       if (logger_dataAvailable() && UARTTxEmpty) {
         uint8_t iCounter;
         // fill transmit FIFO with 14 bytes
         for (iCounter = 0; iCounter < 14 && logger_dataAvailable(); iCounter++) {
            UARTSendByte(logger_read());
         }
       }

       rdm630_process(msTicks);

       if (rdm630_data_available()) {
    	   uint32_t rfid_id = rdm630_read_rfid_id();
    	   rdm630_reset();
    	   logger_logNumberln(rfid_id);
       }

   }
}

/*********************************************************************************
**                            End Of File
*********************************************************************************/
