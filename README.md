# ASF4Template
A template project for using Atmel START generated ASF4 projects.

# Windows Setup
1. Install [Chocolatey](https://chocolatey.org/install#installing-chocolatey)
2. Run `choco install make` from a shell with administrator priveledges (right click -> Run as administrator)
3. Download and extract the [Atmel ARM GNU toolchain](https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers) and add it to your path (e.g. Control Panel -> System -> Advanced system settings -> Environment variables -> Path -> Edit -> New -> `C:\Users\<username>\<path>\arm-none-eabi\bin`).
4. Download and extract OpenOCD and add it to your path (e.g. `C:\Users\<username>\<path>\openocd-0.10.0\bin`).
