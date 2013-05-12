#include "LPC13xx.h"
#include "gpio.h"
#include "ext_int.h"
#include "softuart.h"
#include "logger.h"

/**
 * It's important to define the correct interrupt handler here (e.g. in this example PORT0 is used)
 */
void PIOINT0_IRQHandler(void) {


	if ( GPIOIntStatus( SOFTUART_PORT, SOFTUART_RX_PIN ) )  {

		// start receiving bits
		softuart_start();
		//logger_logStringln("interrupt");

		GPIOIntClear( SOFTUART_PORT, SOFTUART_RX_PIN );
	}

	return;
}
