#include <atmel_start.h>

#define digitalWrite(pin, val) gpio_set_pin_level(pin, val)
#define pinMode(pin, val) gpio_set_pin_function(pin, val)

char spiSendReceive(char send) {
    struct io_descriptor *io;
    spi_m_sync_get_io_descriptor(&SPI_0, &io);
    spi_m_sync_enable(&SPI_0);
    /* Control the slave select (SS) pin */
    gpio_set_pin_level(SPI_0_SS, false);
    io_write(io, send, 1);

    char data;
    if (io_read(io, (uint8_t *)&data, 1) == 1) {
        /* read OK, handle data. */;
        return data;
    } else {
        // Error - in real life, actually handle this, but for now...
        return 0x00;
    }
    /* Control the slave select (SS) pin */
    gpio_set_pin_level(SPI_0_SS, true);
}

int main() {
    atmel_start_init();
    /* Replace with your application code */
    while (1) {
        SPI_0_example();
        delay_ms(100);
    }
}