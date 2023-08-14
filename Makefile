# Makefile for bare-metal SAMD21 ARM Cortex-M0+ programming

# We'll pick up the GCC toolchain from the Arduino installation
ifeq ($(OS),Windows_NT)
ARDUINO=/Users/John.Honniball/AppData/Local/Arduino15
else
ARDUINO=/home/john/.arduino15
endif

# We need CMSIS from the Arduino installation
CMSISDIR=$(ARDUINO)/packages/arduino/tools/CMSIS/4.5.0

# OpenOCD is the Open Source tool used to program the SAMD21
ifeq ($(OS),Windows_NT)
OPENOCD_TOOLS=$(ARDUINO)/packages/arduino/tools/openocd/0.10.0-arduino7/bin
else
OPENOCD_TOOLS=$(ARDUINO)/packages/arduino/tools/openocd/0.10.0-arduino7/bin
endif

MCU=cortex-m0plus
SAMMCU=__SAMD21G17A__

ARM_TOOLS=$(ARDUINO)/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/bin

CC=$(ARM_TOOLS)/arm-none-eabi-gcc
LD=$(ARM_TOOLS)/arm-none-eabi-gcc
OC=$(ARM_TOOLS)/arm-none-eabi-objcopy
SZ=$(ARM_TOOLS)/arm-none-eabi-size

OPENOCD=$(OPENOCD_TOOLS)/openocd

LDSCRIPT=$(ARDUINO)/packages/arduino/hardware/samd/1.8.13/variants/arduino_mzero/linker_scripts/gcc/flash_without_bootloader.ld
STARTUP=$(CMSISDIR)/Device/ARM/ARMCM0plus/Source/GCC/startup_ARMCM0plus.S
SYSTEM=$(ARDUINO)/packages/arduino/tools/CMSIS-Atmel/1.2.0/CMSIS/Device/ATMEL/samd21/source/system_samd21.c

CFLAGS=-Wall -mthumb -mfloat-abi=soft -c -o $@ -O3 -D$(SAMMCU) -I$(CMSISDIR)/Device/ARM/ARMCM0plus/Include -I$(CMSISDIR)/CMSIS/Include -I$(ARDUINO)/packages/arduino/tools/CMSIS-Atmel/1.2.0/CMSIS/Device/ATMEL
LDFLAGS=-mthumb -mfloat-abi=soft --specs=nosys.specs -o $@ -T$(LDSCRIPT)
OCFLAGS=-R .stack -O binary
SZFLAGS=-B -d
INFOFLAGS=-c something

OBJS=blinky.o
ELFS=$(OBJS:.o=.elf)
BINS=$(OBJS:.o=.bin)

# Default target will compile and link all C sources, but not program anything
all: $(BINS)
.PHONY: all

blinky.bin: blinky.elf
	$(OC) $(OCFLAGS) blinky.elf blinky.bin

blinky.elf: blinky.o startup_ARMCM0plus.o system_samd21.o
	$(LD) -mcpu=$(MCU) $(LDFLAGS) startup_ARMCM0plus.o system_samd21.o blinky.o
	$(SZ) $(SZFLAGS) blinky.elf
	
blinky.o: blinky.c
	$(CC) -mcpu=$(MCU) $(CFLAGS) blinky.c

system_samd21.o: $(SYSTEM)
	$(CC) -mcpu=$(MCU) $(CFLAGS) -I$(ARDUINO)/packages/arduino/tools/CMSIS-Atmel/1.2.0/CMSIS/Device/ATMEL/samd21/include $(SYSTEM)

startup_ARMCM0plus.o: $(STARTUP)
	$(CC) -mcpu=$(MCU) $(CFLAGS) $(STARTUP)

# Target to invoke the programmer and program the flash memory of the MCU
prog: blinky.bin
	$(OPENOCD) -c write blinky.bin 0x8000000

.PHONY: prog

# Target 'testprog' will connect to the programmer and read the
# device ID, but not program it
testprog:
	$(OPENOCD) $(INFOFLAGS)

.PHONY: testprog

# Target 'clean' will delete all object files, ELF files, and BIN files
clean:
	-rm -f $(OBJS) $(ELFS) $(BINS) startup_ARMCM0plus.o system_samd21.o

.PHONY: clean


