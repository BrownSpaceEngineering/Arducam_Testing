# bse-fsw-template
A template project for BSE flight software projects. Built to work with Atmel START generated ASF4 projects, on the Atmel SAMD21J18A processor.

Note: the Atmel START library in this repo is the bearbones minimal configuration; projects using this template should add required components and libraries.

## Building
Run `make`. The produced binaries will end up in `build/`

## Uploading to, running & debugging microcontroller
To upload your code to the microcontroller, and then run or debug it using gdb:
1. Make sure you've built the code with `make`
2. Make sure you're connected to the board or debugger
3. From within this project directory, start `openocd`
4. In a new terminal within this project directory, run `make connect` (it's fine if an error about "No executable specified" comes up)
5. At the `(gdb)` prompt run `load` to upload the binary to the microcontroller
6. Run `monitor reset halt` to reset the microcontroller and start debugging
7. Type `c` to run (continue) the program
   - If you want to step through your code, set any breakpoints you need before this (ex. via `break main`)

Tips: 
- To halt the code and get back to the debugger, do `Ctrl-C`
- To rebuild the code without closing gdb, run `make` at the `(gdb)` prompt and repeat steps 5-7

### Using Visual Studio Code
You can use the vscode debugger interface instead of the gdb textual interface to debug programs by installing the [Cortex-Debug](https://github.com/Marus/cortex-debug) vscode extension from the marketplace.

To upload, run and debug using vscode:
1. Make sure you've built the code with `make`
2. Make sure you're connected to the board or debugger
3. Under the "Run and Debug" vscode menu on the left, click the green arrow to start debugging
   - Make sure the "Debug (OpenOCD)" configuration is selected in the dropdown to the right of the arrow
4. Wait for the debugger to start up (your bottom bar will turn orange once it does)
   - If it doesn't start up, check the "Output" and "Debug Console" panes of vscode for error messages
5. On the hovering toolbar that comes up, click the play button to run (continue) the program
   - If you want to step through your code, set any breakpoints you need before this using the interface to the left of any code file you have open in vscode

## Installing the Toolchain
### Windows
1. Install [Chocolatey](https://chocolatey.org/install#installing-chocolatey)
2. Run `choco install make` from a shell with administrator priveledges (right click -> Run as administrator)
3. Download and extract the "Arm GNU Toolchain for 32-bit Devices" for Windows from [here](https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers).
   - The Microchip link asks you to make an account, so either ask your BSE leader for the BSE account credentials or create your own account.
   - To avoid dealing with an account AND to get some newer GCC and (importantly) GDB features, use [this link](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) instead.
   - Make sure to add the bin folder to your path (e.g. Control Panel -> System -> Advanced system settings -> Environment variables -> Path -> Edit -> New -> `C:\Users\<username>\<path>\arm-none-eabi\bin`).
4. Download and extract the newest Windows version of OpenOCD from [here](https://github.com/xpack-dev-tools/openocd-xpack/releases).
   - Make sure to add the bin folder to your path (e.g. `C:\Users\<username>\<path>\openocd-0.10.0\bin`). 
   - If you have trouble with that version you can also use [this site](http://www.freddiechopin.info/en/download/category/4-openocd) (you'll need something like [7-Zip](https://www.7-zip.org/)).

### Mac OSX
1. Install homebrew [here](https://brew.sh/)
2. Install ARM developer tools: 
   ```
   brew tap PX4/homebrew-px4
   brew update
   brew search px4
   brew install gcc-arm-none-eabi-80
   ```
   If the last command fails but tells you to "Install the Command Line Tools" using `xcode-select --install`, do so.

3. Install openocd: `brew install openocd`

### Linux (Ubuntu)
1. Install the "Arm GNU Toolchain for 32-bit Devices" for Linux from [here](https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers).
   - The Microchip link asks you to make an account, so either ask your BSE leader for the BSE account credentials or create your own account.
   - To avoid dealing with an account AND to get some newer GCC and (importantly) GDB features, use [this link](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) instead. You may need to use the second-to-latest version if you can't find a 64-bit linux installer (i.e. "64-bit Tarball").
   - Make sure to add (or symlink) the bin folder into your path (e.g. add `export PATH=/path/to/arm-none-eabi/bin/:$PATH` to the end of `~/.bashrc` and restart your terminal or run `source ~/.bashrc`).
2. Install openocd: `sudo apt install openocd`. Unfortunately this doesn't seem to add required udev rules, so you'll need to:
    1. Download the udev rules: `wget https://repo.or.cz/openocd.git/blob_plain/HEAD:/contrib/60-openocd.rules`
    2. Set the rules' owner to root: `sudo chown root:root 60-openocd.rules`
    3. Set the rules' permissions: `sudo chmod 644 60-openocd.rules`
    4. Move the rules: `sudo mv 60-openocd.rules /etc/udev/rules.d/`
    5. Add yourself to the plugdev group: `sudo usermod -a -G plugdev $USERNAME`
    6. Probably log out or reboot. Just running `sudo udevadm control --reload` might also work.

### Linux (other distributions)
1. Install the Arm GNU Toolchain as above.
2. openocd is probably in your package manager, but if not there are instructions [here](http://openocd.org/getting-openocd/). Make sure to put (or symlink) the bin folder in your path.
2. You may need to follow the instructions above to add the udev rules.

## Changing ASF library configuration
Follow this procedure if you'd like to add Atmel Start ASF libraries, change chip configuration or pinout, etc.
1. Go to [start.atmel.com](https://start.atmel.com)
2. Click on "Load project from file" and upload `./asf-samd21/atmel_start_config.atstart`
4. As desired, add software components, configure pinmux, package, or clocks
5. Click "Export project", make sure "Makefile (standalone)" is selected, and click "Download pack"
8. Change the `.atzip` file extension on the file you downloaded to `.zip` and extract it
10. Delete the `main.c` file in the downloaded directory
11. Delete the contents of `./asf-samd21/` and copy all files in the downloaded directory there
12. Modify `ATMEL_SRC_DIRS` and `ATMEL_INCLUDE_DIRS` in `./Makefile` according to the instructions there. You may need to make further modifications in `./Makefile` based on changes made by Atmel Start to `./asf-samd21/gcc/Makefile` (to see what changed, do `git diff ./asf-samd21/gcc/Makefile`).
