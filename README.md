# Arducam_Testing

This is the testing repository for camera code. Important components:

* `serial_listener.c`: contains the client code for communicating with a board running the camera testing code
* Branch `atmel_arducam_poc`: contains a minimally modified variation of Atmel's Raspberry Pi/ArduCAM sample code capable of running on the SAMD21
* Branch `driver_development`: contains work-in-progress development efforts toward a cleaned-up driver based on the sample code (at the moment, the I<sup>2</sup>C code is iffy)
