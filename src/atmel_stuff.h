#ifndef BSE_FSW_TEMPLATE_ATMEL_STUFF_H
#define BSE_FSW_TEMPLATE_ATMEL_STUFF_H

// If we're getting garbage, maybe reverse these?
#define INPUT 0
#define OUTPUT 1

#define HIGH  1
#define LOW   0

#define digitalWrite(pin, val) gpio_set_pin_level(pin, val)
#define pinMode(pin, val) gpio_set_pin_function(pin, val)
char spiSendReceive(char send);

#endif //BSE_FSW_TEMPLATE_ATMEL_STUFF_H
