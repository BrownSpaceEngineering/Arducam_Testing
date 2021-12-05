#include <atmel_start.h>
#include "ArduCAM.h"
#include "sccb_bus.h"

int main(void)
{
    // Ports we need to be thinking about:
    // CAM_CS1, scl_port, sda_port
    // SPI_0 ports are defined in the Atmel Start configuration (I think)

	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

    // Configure the chip select pin
    // Parrot https://ww1.microchip.com/downloads/en/DeviceDoc/00002465A.pdf
    gpio_set_pin_direction(CAM_CS1, GPIO_DIRECTION_OUT);
    gpio_set_pin_pull_mode(CAM_CS1, GPIO_PULL_OFF);
    gpio_set_pin_level(CAM_CS1, true);

    // ArduCAM initialization
    ArduCAM_CS_init(CAM_CS1, -1, -1, -1);
    sccb_bus_init();
    // We shouldn't need to call *_sync_init for any service defined in the Atmel Start config -- ateml_start_init
    // will take care of all of those for us
    Arducam_bus_detect(CAM_CS1, -1, -1, -1);
    resetFirmware(CAM_CS1, -1, -1, -1);
    ArduCAM_Init(sensor_model);

    // Take photo
    singleCapture(CAM_CS1);

    // Send over UART
    struct io_descriptor *io;
    usart_sync_get_io_descriptor(&USART_0, &io);
    usart_sync_enable(&USART_0);
    // Testing code to verify USART is working (it is!)
//    while (1) {
//        io_write(io, (uint8_t *) "hello\n", 6);
//        delay_ms(1000);
//    }
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
