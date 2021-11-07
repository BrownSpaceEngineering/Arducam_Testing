#include <atmel_start.h>
#include "ArduCAM.h"
#include "sccb_bus.h"

int main(void)
{
    // Ports we need to be thinking about:
    // CAM_CS1, scl_port, sda_port

	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

    // ArduCAM initialization
    ArduCAM_CS_init(CAM_CS1, -1, -1, -1);
    sccb_bus_init();
    // if something SPI-related doesn't work, try calling this:
    // spi_m_sync_init()
    Arducam_bus_detect(CAM_CS1, -1, -1, -1);
    resetFirmware(CAM_CS1, -1, -1, -1);
    ArduCAM_Init(sensor_model);

    // Take photo
    singleCapture(CAM_CS1);

    // Send over UART
    struct io_descriptor *io;
    usart_sync_get_io_descriptor(&USART_0, &io);
    usart_sync_enable(&USART_0);
    io_write(io, (uint8_t *)readbuf, length);

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
