MCU=atmega32u4
MCU2=m32u4
F_CPU=16000000
F_USB=16000000
ARCH=AVR8
CC=avr-gcc
OBJCOPY=avr-objcopy
# CFLAGS= -Wall -g -O2 -mmcu=$(MCU) -DF_CPU=$(F_CPU)
LUFA_PATH=LUFA
CC_FLAGS= -DUSE_LUFA_CONFIG_HEADER -IConfig/
LD_FLAGS = $(PRINTF_LIB) $(SCANF_LIB) $(MATH_LIB)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt
PRINTF_LIB = $(PRINTF_LIB_FLOAT)
SCANF_LIB_FLOAT = -Wl,-u,vfscanf -lscanf_flt
SCANF_LIB = $(SCANF_LIB_FLOAT)
OPTIMIZATION = s
TARGET=cmiv2029
SRC=main.c serial.c spi.c i2c.c Descriptors.c $(LUFA_SRC_USB) $(LUFA_SRC_USBCLASS)

all:
	# $(CC) $(CFLAGS) $(LD_FLAGS) -o $(TARGET).bin $(SRC)
	# $(OBJCOPY) -j .text -j .data -O ihex $(TARGET).bin $(TARGET).hex

fuses:
	avrdude -p $(MCU2) -c avrisp2 -P /dev/ttyS4 -b 19200 -U efuse:w:0xFD:m -U hfuse:w:0xD6:m -U lfuse:w:0xFF:m
	# avrdude -p $(MCU2) -c avrisp2 -P /dev/ttyS5 -b 19200 -U efuse:w:0xFD:m -U hfuse:w:0xD9:m -U lfuse:w:0xE2:m

flash:
	avrdude -p $(MCU2) -c avrisp2 -P /dev/ttyS4 -b 19200 -U flash:w:$(TARGET).hex:i

eeprom:
	avrdude -p $(MCU2) -c avrisp2 -P /dev/ttyS4 -b 19200 -U eeprom:w:$(TARGET).eep:i

clean:
	rm -f *.o *.bin *.hex


# Include LUFA-specific DMBS extension modules
DMBS_LUFA_PATH ?= $(LUFA_PATH)/Build/LUFA
include $(DMBS_LUFA_PATH)/lufa-sources.mk
include $(DMBS_LUFA_PATH)/lufa-gcc.mk

# Include common DMBS build system modules
DMBS_PATH      ?= $(LUFA_PATH)/Build/DMBS/DMBS
include $(DMBS_PATH)/core.mk
include $(DMBS_PATH)/cppcheck.mk
include $(DMBS_PATH)/doxygen.mk
include $(DMBS_PATH)/dfu.mk
include $(DMBS_PATH)/gcc.mk
include $(DMBS_PATH)/hid.mk
include $(DMBS_PATH)/avrdude.mk
include $(DMBS_PATH)/atprogram.mk


