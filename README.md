![Static Badge](https://img.shields.io/badge/MCU-ATSAMD21-green "MCU:ATSAMD21")
![Static Badge](https://img.shields.io/badge/IDE-MPLAB_X_V6.20-green "IDE:MPLAB_X_V6.20")
![Static Badge](https://img.shields.io/badge/BOARD-SAMD21_Curiosity_Nano-green "BOARD:SAMD21_Curiosity_Nano")

# SAMD21blinky #

Some simple SAMD21 ARM test programs to verify toolchain, programmer, and chip.

The programs are in C and may be compiled with GCC and 'make' on Linux.
Alternatively, Microchip MPLAB X and 'xc32' may be used on Linux or Windows.

Note that access to the chip registers in the source must be coded differently
between GCC and MPLAB X.
For the differences, compare 'blinky.c' in the root of the repo (GCC) with
'blinky.c' in the subdirectory 'blinky.X' (MPLAB X).
The registers themselves are of course the same,
but the way that they're named and the data structures used to access them are
different.

## Chips Supported ##

At present, there's only support for the SAMD21G17D on the SAMD21 Curiosity Nano.

I have a SAMD21E18A on a break-out board that I'd like to get working too.

## ARM Toolchains ##

### MPLAB X ###

A recent version of MPLAB X and the 'xc32' compiler.
The source code and IDE setup files are in the subdirectory 'blinky.X'.
Tested with MPLAB X V6.20 and xc32 V4.60.

### GCC and 'make' ###

We can borrow the compiler, header files, startup files, and linker scripts
from the Arduino installation.
Source code 'blinky.c' and the usual 'Makefile' in the root of the repo.
Only compilation and linking is working at the moment;
programming the chip is not yet possible
(what programming tool should we use?).
Also, the 'blinky.c' file is at an early stage of development and does not
set up very much of the hardware yet.

