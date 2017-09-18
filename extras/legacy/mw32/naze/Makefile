# Makefile for Hackflight on Naze32 boards
#
# Requires: 
#
#      https://github.com/simondlevy/BreezySTM32
#      https://github.com/simondlevy/MPU6050
#      https://github.com/simondlevy/SpektrumDSM
#
#   This file is part of Hackflight.
#
#   Hackflight is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   Hackflight is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#   You should have received a copy of the GNU General Public License
#   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.

# Change this to wherever you put Hackflight
HACKFLIGHT_DIR = $(HOME)/Desktop/Hackflight

# Change this to wherever you put BreezySTM32
BREEZY_DIR = $(HOME)/Desktop/BreezySTM32

# Change this to where you put your Arduino libraries
ARDUINO_DIR = $(HOME)/Development/arduino-1.8.4/libraries

###############################################################################

FIRMWARE_DIR = $(HACKFLIGHT_DIR)/legacy/mw32

F1_DIR = $(BREEZY_DIR)/f1

SPEKTRUM_DIR = $(ARDUINO_DIR)/SpektrumDSM/src

MPU6050_DIR = $(ARDUINO_DIR)/MPU6050/src

# Debugger optons, must be empty or GDB
DEBUG ?=

# Serial port/Device for flashing
SERIAL_DEVICE	?= /dev/ttyUSB0

# Working directories
ROOT		 = $(dir $(lastword $(MAKEFILE_LIST)))
SRC_DIR		 = $(ROOT)
CMSIS_DIR	 = $(BREEZY_DIR)/lib/CMSIS
STDPERIPH_DIR	 = $(BREEZY_DIR)/lib/STM32F10x_StdPeriph_Driver
OBJECT_DIR	 = $(ROOT)/obj
BIN_DIR		 = $(ROOT)/obj

hackflight_CSRC = \
		   $(F1_DIR)/startup_stm32f10x_md_gcc.S \
		   $(F1_DIR)/drv_gpio.c \
		   $(F1_DIR)/drv_system.c \
		   $(F1_DIR)/drv_serial.c \
		   $(F1_DIR)/drv_uart.c \
		   $(F1_DIR)/drv_timer.c \
		   $(CMSIS_SRC) \
		   $(STDPERIPH_SRC)

VPATH		:= $(SRC_DIR):$(SRC_DIR)/startups

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
			               $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src
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
INCLUDE_DIRS = $(SRC_DIR) \
		   $(SRC_DIR)/.. \
		   $(BREEZY_DIR) \
		   $(FIRMWARE_DIR) \
		   $(F1_DIR) \
		   $(SPEKTRUM_DIR) \
		   $(MPU6050_DIR) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM3/CoreSupport \
		   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3

ifeq ($(DEBUG),GDB)
OPTIMIZE	 = -Og

else
OPTIMIZE	 = -Os
LTO_FLAGS	 = -flto -fuse-linker-plugin $(OPTIMIZE)
endif

DEBUG_FLAGS	 = -ggdb3

CFLAGS		 = $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
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

LD_SCRIPT	 = $(F1_DIR)/stm32_flash.ld
LDFLAGS		 = -lm \
		   -nostartfiles \
		   -lc \
		   --specs=rdimon.specs \
		   $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(DEBUG_FLAGS) \
		   -static \
		   -Wl,-gc-sections,-Map,$(TARGET_MAP) \
		   -T$(LD_SCRIPT)

#
# Things we will build
#

TARGET_HEX	 = $(BIN_DIR)/hackflight.hex
TARGET_ELF	 = $(BIN_DIR)/hackflight.elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/hackflight/,$(basename $(hackflight_CSRC)))) \
			   $(OBJECT_DIR)/hackflight/Arduino.o \
			   $(OBJECT_DIR)/hackflight/HardwareWire.o \
			   $(OBJECT_DIR)/hackflight/MPU6050.o \
			   $(OBJECT_DIR)/hackflight/SpektrumDSM.o \
			   $(OBJECT_DIR)/hackflight/Motor.o \
			   $(OBJECT_DIR)/hackflight.o
TARGET_MAP   = $(OBJECT_DIR)/hackflight.map

# List of buildable ELF files and their object dependencies.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(C++) -o $@ $^ $(LDFLAGS)

MKDIR_OBJDIR = @mkdir -p $(dir $@)

# Compile
$(OBJECT_DIR)/hackflight/%.o: %.c
	$(MKDIR_OBJDIR)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/hackflight/%.o: %.S
	$(MKDIR_OBJDIR)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 

# C++ support
%.o: %.cpp
	$(CC) -c $(CFLAGS) -o $@ $^

$(OBJECT_DIR)/hackflight/Arduino.o: $(F1_DIR)/Arduino.cpp $(BREEZY_DIR)/Arduino.h
	@$(CC) -c -o $(OBJECT_DIR)/hackflight/Arduino.o $(CFLAGS) $(F1_DIR)/Arduino.cpp

$(OBJECT_DIR)/hackflight/HardwareWire.o: $(F1_DIR)/HardwareWire.cpp $(BREEZY_DIR)/Wire.h
	@$(CC) -c -o $(OBJECT_DIR)/hackflight/HardwareWire.o $(CFLAGS) $(F1_DIR)/HardwareWire.cpp

$(OBJECT_DIR)/hackflight/MPU6050.o: $(MPU6050_DIR)/MPU6050.cpp $(MPU6050_DIR)/MPU6050.h
	@$(CC) -c -o $(OBJECT_DIR)/hackflight/MPU6050.o $(CFLAGS) $(MPU6050_DIR)/MPU6050.cpp

$(OBJECT_DIR)/hackflight/SpektrumDSM.o: $(SPEKTRUM_DIR)/SpektrumDSM.cpp $(SPEKTRUM_DIR)/SpektrumDSM.h
	@$(CC) -c -o $(OBJECT_DIR)/hackflight/SpektrumDSM.o $(CFLAGS) $(SPEKTRUM_DIR)/SpektrumDSM.cpp

$(OBJECT_DIR)/hackflight/Motor.o: $(F1_DIR)/Motor.cpp
	@$(CC) -c -o $(OBJECT_DIR)/hackflight/Motor.o $(CFLAGS) $(F1_DIR)/Motor.cpp

$(OBJECT_DIR)/hackflight.o: ./hackflight.cpp naze.hpp $(FIRMWARE_DIR)/*.hpp
	@$(C++) -c -std=gnu++11 -o $(OBJECT_DIR)/hackflight.o $(CFLAGS) ./hackflight.cpp

clean:
	rm -rf $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP) *.o obj 

flash_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	echo -n 'R' >$(SERIAL_DEVICE)
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

flash: flash_$(TARGET)

unbrick_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

unbrick: unbrick_$(TARGET)

listen:
	miniterm.py $(SERIAL_DEVICE) 115200
