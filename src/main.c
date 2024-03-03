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

    // Stuff copied from https://www.avrfreaks.net/forum/asf4-spi-cs-problem
//    spi_m_sync_set_mode(&SPI_0,SPI_MODE_3); //set the spi to mode 3 (sets the clock/data phases)
//    spi_m_sync_set_baudrate(&SPI_0,1000000); //set baudrate to 10Mhz  NOTE: 1000000 is actually 10Mhz write, dont' know why have to chck this
//    spi_m_sync_set_char_size(&SPI_0,SPI_CHAR_SIZE_8);  //8 bit character size to write
//    spi_m_sync_set_data_order(&SPI_0,SPI_DATA_ORDER_MSB_1ST);

    // Configure the chip select pin
    // Parrot https://ww1.microchip.com/downloads/en/DeviceDoc/00002465A.pdf
    // (I think ArduCAM_CS_init does most of this for us, but can't hurt to be sure)
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

    REG_PORT_DIR1 |= (1<<30);

    while (1) {
        REG_PORT_OUT1 &= ~(1<<30);
        // Take photo
        singleCapture(CAM_CS1);
        REG_PORT_OUT1 |= (1<<30);

        // Send over UART
        struct io_descriptor *io;
//    usart_sync_set_baud_rate(&USART_0, 9600);
        usart_sync_get_io_descriptor(&USART_0, &io);
        usart_sync_enable(&USART_0);
        // Testing code to verify USART is working (it is!)
//    while (1) {
//        io_write(io, (uint8_t *) "hello\n", 6);
//        delay_ms(1000);
//    }

        uint8_t header[] = {0x75, 0x03, 0x30, 0x75, 0x03, 0x30};
        io_write(io, (uint8_t *) header, sizeof(header));

        // Send the image size -- MSBs first
        // (N.b. this can only handle images up to ~65 KB, which may not be large enough)
        uint8_t len_msbs = (length >> 8) & 0xff;
        uint8_t len_lsbs = length & 0xff;
        io_write(io, &len_msbs, 1);
        io_write(io, &len_lsbs, 1);

        int32_t bytes_written = io_write(io, (uint8_t *) readbuf, length);
        if (bytes_written != length) {
            ASSERT(0);
        }
    }
//    for (int i = 0; i < length; i++) {
//        io_write(io, (uint8_t *)(readbuf + i), 1);
//    }
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
