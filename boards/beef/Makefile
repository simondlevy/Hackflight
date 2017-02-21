###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################

# Change this to wherever you put hackflight
HACKFLIGHT_DIR = $(HOME)/Desktop/hackflight

# Change this to wherever you put Arduino libraries
ARDUINO_DIR = $(HOME)/Development/arduino-1.6.13/hardware/teensy/avr/libraries

## Change this to wherever you put BreezySTM32
BREEZY_DIR = $(HOME)/Desktop/BreezySTM32

###############################################################################
# Things that the user might override on the commandline
#

F1_DIR = $(BREEZY_DIR)/f1

MPU6050_DIR = $(ARDUINO_DIR)/MPU6050

SPEKTRUM_DIR = $(ARDUINO_DIR)/SpektrumDSM

TARGET ?= BEEF

# Compile-time options
OPTIONS		?=

# Debugger optons, must be empty or GDB
DEBUG ?=

# Serial port/Device for flashing
SERIAL_DEVICE	?= /dev/ttyUSB0

###############################################################################
# Things that need to be maintained as the source changes
#

# Working directories
ROOT		 = ..
HERE         = .
SRC_DIR		 = $(ROOT)
OBJECT_DIR	 = $(HERE)/obj
BIN_DIR		 = $(HERE)/obj
CMSIS_DIR	 = $(BREEZY_DIR)/lib/CMSIS
STDPERIPH_DIR = $(BREEZY_DIR)/lib/STM32F10x_StdPeriph_Driver

# Source files common to all targets
$(TARGET)_SRC = $(CMSIS_SRC) $(STDPERIPH_SRC)

# In some cases, %.s regarded as intermediate file, which is actually not.
# This will prevent accidental deletion of startup code.
.PRECIOUS: %.s

# Search path for hackflight sources
VPATH		:= $(SRC_DIR):#$(SRC_DIR)/hackflight_startups

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
			               $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(STDPERIPH_DIR):$(STDPERIPH_DIR)/src
STDPERIPH_SRC	 = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

###############################################################################
# Things that might need changing to use different tools
#

# Tool names
C++		 = arm-none-eabi-g++
CC		 = arm-none-eabi-gcc
OBJCOPY	 = arm-none-eabi-objcopy

#
# Tool options.
#
INCLUDE_DIRS = . \
			   $(BREEZY_DIR) \
			   $(F1_DIR) \
			   $(MPU6050_DIR) \
			   $(SPEKTRUM_DIR) \
			   $(STDPERIPH_DIR)/inc \
			   $(CMSIS_DIR)/CM3/CoreSupport \
			   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3

ifeq ($(DEBUG),GDB)
OPTIMIZE	 = -Og
LTO_FLAGS	 = $(OPTIMIZE)
else
OPTIMIZE	 = -Os
LTO_FLAGS	 = -flto -fuse-linker-plugin $(OPTIMIZE)
endif

DEBUG_FLAGS	 = -ggdb3

CFLAGS	 = $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   $(DEBUG_FLAGS) \
		   -Wall -pedantic -Wextra -Wshadow -Wunsafe-loop-optimizations \
		   -ffunction-sections \
		   -fdata-sections \
		   -DSTM32F10X_MD \
		   -DUSE_STDPERIPH_DRIVER

ASFLAGS		 = $(ARCH_FLAGS) \
		   -x assembler-with-cpp \
		   $(addprefix -I,$(INCLUDE_DIRS))

# XXX Map/crossref output?
LD_SCRIPT	 = $(F1_DIR)/stm32_flash.ld
LDFLAGS	 = -lm \
		   -nostartfiles \
		   -lc \
  		   --specs=rdimon.specs \
		   $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(DEBUG_FLAGS) \
		   -static \
		   -Wl,-gc-sections,-Map,$(TARGET_MAP) \
		   -T$(LD_SCRIPT)

###############################################################################
# No user-serviceable parts below
###############################################################################

#
# Things we will build
#

TARGET_HEX	 = $(BIN_DIR)/hackflight_$(TARGET).hex
TARGET_ELF	 = $(BIN_DIR)/hackflight_$(TARGET).elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC)))) \
			   $(OBJECT_DIR)/hackflight.o \
			   $(OBJECT_DIR)/imu.o \
			   $(OBJECT_DIR)/mixer.o \
			   $(OBJECT_DIR)/msp.o \
			   $(OBJECT_DIR)/rc.o \
			   $(OBJECT_DIR)/stabilize.o \
			   $(OBJECT_DIR)/filters.o \
			   $(OBJECT_DIR)/board.o \
			   $(OBJECT_DIR)/Arduino.o \
			   $(OBJECT_DIR)/SpektrumDSM.o \
			   $(OBJECT_DIR)/HardwareWire.o \
			   $(OBJECT_DIR)/Motor.o \
			   $(OBJECT_DIR)/MPU6050.o \
			   $(OBJECT_DIR)/drv_system.o \
			   $(OBJECT_DIR)/drv_serial.o \
			   $(OBJECT_DIR)/drv_gpio.o \
			   $(OBJECT_DIR)/drv_uart.o \
			   $(OBJECT_DIR)/drv_timer.o \
			   $(OBJECT_DIR)/startup_stm32f10x_md_gcc.o

TARGET_MAP   = $(OBJECT_DIR)/hackflight_$(TARGET).map

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(C++) -o $@ $^ $(LDFLAGS)

MKDIR_OBJDIR = @mkdir -p $(dir $@)

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(MKDIR_OBJDIR)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	$(MKDIR_OBJDIR)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 

# C++ support

FIRMDIR = $(HACKFLIGHT_DIR)/firmware

$(OBJECT_DIR)/hackflight.o: $(FIRMDIR)/hackflight.cpp $(FIRMDIR)/hackflight.hpp
	@$(CC) -c -o $(OBJECT_DIR)/hackflight.o $(CFLAGS) $(FIRMDIR)/hackflight.cpp

$(OBJECT_DIR)/imu.o: $(FIRMDIR)/imu.cpp $(FIRMDIR)/imu.hpp
	@$(CC) -c -o $(OBJECT_DIR)/imu.o $(CFLAGS) $(FIRMDIR)/imu.cpp

$(OBJECT_DIR)/mixer.o: $(FIRMDIR)/mixer.cpp $(FIRMDIR)/mixer.hpp
	@$(CC) -c -o $(OBJECT_DIR)/mixer.o $(CFLAGS) $(FIRMDIR)/mixer.cpp

$(OBJECT_DIR)/msp.o: $(FIRMDIR)/msp.cpp $(FIRMDIR)/msp.hpp
	@$(CC) -c -o $(OBJECT_DIR)/msp.o $(CFLAGS) $(FIRMDIR)/msp.cpp

$(OBJECT_DIR)/rc.o: $(FIRMDIR)/rc.cpp $(FIRMDIR)/rc.hpp
	@$(CC) -c -o $(OBJECT_DIR)/rc.o $(CFLAGS) $(FIRMDIR)/rc.cpp

$(OBJECT_DIR)/stabilize.o: $(FIRMDIR)/stabilize.cpp $(FIRMDIR)/stabilize.hpp
	@$(CC) -c -o $(OBJECT_DIR)/stabilize.o $(CFLAGS) $(FIRMDIR)/stabilize.cpp

$(OBJECT_DIR)/filters.o: $(FIRMDIR)/filters.cpp $(FIRMDIR)/filters.hpp
	@$(CC) -c -o $(OBJECT_DIR)/filters.o $(CFLAGS) $(FIRMDIR)/filters.cpp

$(OBJECT_DIR)/board.o: board.cpp $(FIRMDIR)/board.hpp
	@$(CC) -I$(FIRMDIR) -c -o $(OBJECT_DIR)/board.o $(CFLAGS) board.cpp

$(OBJECT_DIR)/Arduino.o: $(F1_DIR)/Arduino.cpp
	@$(CC) -c -o $(OBJECT_DIR)/Arduino.o $(CFLAGS) $(F1_DIR)/Arduino.cpp

$(OBJECT_DIR)/HardwareWire.o: $(F1_DIR)/HardwareWire.cpp
	@$(CC) -c -o $(OBJECT_DIR)/HardwareWire.o $(CFLAGS) $(F1_DIR)/HardwareWire.cpp

$(OBJECT_DIR)/drv_system.o: $(F1_DIR)/drv_system.c
	@$(CC) -c -o $(OBJECT_DIR)/drv_system.o $(CFLAGS) $(F1_DIR)/drv_system.c

$(OBJECT_DIR)/drv_i2c.o: $(F1_DIR)/drv_i2c.c
	@$(CC) -c -o $(OBJECT_DIR)/drv_i2c.o $(CFLAGS) $(F1_DIR)/drv_i2c.c

$(OBJECT_DIR)/drv_serial.o: $(F1_DIR)/drv_serial.c
	@$(CC) -c -o $(OBJECT_DIR)/drv_serial.o $(CFLAGS) $(F1_DIR)/drv_serial.c

$(OBJECT_DIR)/drv_gpio.o: $(F1_DIR)/drv_gpio.c
	@$(CC) -c -o $(OBJECT_DIR)/drv_gpio.o $(CFLAGS) $(F1_DIR)/drv_gpio.c

$(OBJECT_DIR)/drv_uart.o: $(F1_DIR)/drv_uart.c
	@$(CC) -c -o $(OBJECT_DIR)/drv_uart.o $(CFLAGS) $(F1_DIR)/drv_uart.c

$(OBJECT_DIR)/drv_timer.o: $(F1_DIR)/drv_timer.c
	@$(CC) -c -o $(OBJECT_DIR)/drv_timer.o $(CFLAGS) $(F1_DIR)/drv_timer.c

$(OBJECT_DIR)/startup_stm32f10x_md_gcc.o: $(F1_DIR)/startup_stm32f10x_md_gcc.S
	@$(CC) -c -o $(OBJECT_DIR)/startup_stm32f10x_md_gcc.o $(CFLAGS) $(F1_DIR)/startup_stm32f10x_md_gcc.S

$(OBJECT_DIR)/Motor.o: $(F1_DIR)/Motor.cpp
	@$(CC) -c -o $(OBJECT_DIR)/Motor.o $(CFLAGS) $(F1_DIR)/Motor.cpp

$(OBJECT_DIR)/MPU6050.o: $(MPU6050_DIR)/MPU6050.cpp $(MPU6050_DIR)/MPU6050.h
	@$(CC) -c -o $(OBJECT_DIR)/MPU6050.o $(CFLAGS) $(MPU6050_DIR)/MPU6050.cpp

$(OBJECT_DIR)/SpektrumDSM.o: $(SPEKTRUM_DIR)/SpektrumDSM.cpp $(SPEKTRUM_DIR)/SpektrumDSM.h
	@$(CC) -c -o $(OBJECT_DIR)/SpektrumDSM.o $(CFLAGS) $(SPEKTRUM_DIR)/SpektrumDSM.cpp

clean:
	rm -rf *.o obj $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)

flash_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	echo -n 'R' >$(SERIAL_DEVICE)
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

flash: flash_$(TARGET)

unbrick_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

unbrick: unbrick_$(TARGET)

commit:
	git commit -a --allow-empty-message -m ''
	git push

debug:
	miniterm.py $(SERIAL_DEVICE) 115200

listen:
	miniterm.py $(SERIAL_DEVICE) 115200
