#include <atmel_start.h>
#include "ArduCAM.h"
//#include "ArduCAM.h"

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
    singleCapture(CAM_CS1);

    // Send over UART
    struct io_descriptor *io;
    usart_sync_get_io_descriptor(&USART_0, &io);
    usart_sync_enable(&USART_0);
    io_write(io, (uint8_t *)"Hello World!", 12);

	/* Replace the code below with your application code */


	/* Set pin PB30 to output */
//	REG_PORT_DIR1 |= (1<<30);
//
//	/* Make it blink! */
//	while (1) {
//		REG_PORT_OUT1 &= ~(1<<30);
//		delay_ms(100);
//		REG_PORT_OUT1 |= (1<<30);
//		delay_ms(100);
//		REG_PORT_OUT1 &= ~(1<<30);
//		delay_ms(100);
//		REG_PORT_OUT1 |= (1<<30);
//		delay_ms(100);
//		REG_PORT_OUT1 &= ~(1<<30);
//		delay_ms(100);
//		REG_PORT_OUT1 |= (1<<30);
//		delay_ms(1000);
//	}
}
