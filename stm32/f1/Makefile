# Makefile for Hackflight STM32F1 examples
#
# Copyright (C) 2016 Simon D. Levy
#
# This file is part of Hackflight.
#
# Hackflight is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This code is distributed in the hope that it will be useful,     
# but WITHOUT ANY WARRANTY without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License 
# along with this code.  If not, see <http://www.gnu.org/licenses/>.

# Change this to wherever you put hackflight
HACKFLIGHT_DIR = $(HOME)/Desktop/hackflight

# Change this to wherever you put BreezySTM32
BREEZY_DIR = $(HOME)/Desktop/BreezySTM32

###############################################################################
# Things that the user might override on the commandline
#

TARGET		?= NAZE

CPP_OBJS = hackflight.o imu.o mixer.o msp.o rc.o board.o board_rx.o stabilize.o filters.o filters.o

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
ROOT		  = ..
HERE          = .
SRC_DIR		  = $(ROOT)
OBJECT_DIR	  = $(HERE)/obj
BIN_DIR		  = $(HERE)/obj
F1_DIR        = $(BREEZY_DIR)/f1
DRIVERS_DIR   = $(F1_DIR)/drivers
CMSIS_DIR	  = $(BREEZY_DIR)/lib/CMSIS
STDPERIPH_DIR = $(BREEZY_DIR)/lib/STM32F10x_StdPeriph_Driver
FIRM_DIR       = $(HACKFLIGHT_DIR)/firmware


# Source files common to all targets
NAZE_SRC = $(F1_DIR)/main.c \
           $(F1_DIR)/system.c \
           $(F1_DIR)/system_stm32f10x.c \
           $(F1_DIR)/serial.c \
           $(F1_DIR)/pwm.c \
           $(F1_DIR)/gpio.c \
           $(F1_DIR)/uart_stm32f10x.c \
           $(F1_DIR)/timer.c \
           $(F1_DIR)/startup_stm32f10x_md_gcc.S \
           $(F1_DIR)/i2c_stm32f10x.c \
           $(DRIVERS_DIR)/mpu.c \
           $(DRIVERS_DIR)/ms5611.c \
           $(DRIVERS_DIR)/mb1242.c \
           $(DRIVERS_DIR)/spektrum.c \
           $(CMSIS_SRC) \
           $(STDPERIPH_SRC)

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
CC		 = arm-none-eabi-gcc
OBJCOPY	 = arm-none-eabi-objcopy

#
# Tool options.
#
INCLUDE_DIRS = . \
			   $(FIRM_DIR) \
			   $(BREEZY_DIR) \
			   $(F1_DIR) \
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
		   -DUSE_STDPERIPH_DRIVER \
		   -D$(TARGET) \
		   -DEXTERNAL_DEBUG 

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
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC)))) $(CPP_OBJS)
TARGET_MAP   = $(OBJECT_DIR)/hackflight_$(TARGET).map

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
	mv *.o obj

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

hackflight.o: $(FIRM_DIR)/hackflight.cpp $(FIRM_DIR)/hackflight.hpp $(FIRM_DIR)/rc.hpp
	@$(CC) $(CFLAGS) -c -o hackflight.o $(FIRM_DIR)/hackflight.cpp

imu.o: $(FIRM_DIR)/imu.cpp $(FIRM_DIR)/imu.hpp
	@$(CC) $(CFLAGS) -c -o imu.o $(FIRM_DIR)/imu.cpp

mixer.o: $(FIRM_DIR)/mixer.cpp $(FIRM_DIR)/mixer.hpp $(FIRM_DIR)/rc.hpp
	@$(CC) $(CFLAGS) -c -o mixer.o $(FIRM_DIR)/mixer.cpp

stabilize.o: $(FIRM_DIR)/stabilize.cpp $(FIRM_DIR)/stabilize.hpp $(FIRM_DIR)/stabilize.hpp
	@$(CC) $(CFLAGS) -c -o stabilize.o $(FIRM_DIR)/stabilize.cpp

filters.o: $(FIRM_DIR)/filters.cpp $(FIRM_DIR)/filters.hpp $(FIRM_DIR)/rc.hpp
	@$(CC) $(CFLAGS) -c -o filters.o $(FIRM_DIR)/filters.cpp

msp.o: $(FIRM_DIR)/msp.cpp $(FIRM_DIR)/msp.hpp $(FIRM_DIR)/rc.hpp
	@$(CC) $(CFLAGS) -c -o msp.o $(FIRM_DIR)/msp.cpp

baro.o: $(FIRM_DIR)/baro.cpp $(FIRM_DIR)/baro.hpp
	@$(CC) $(CFLAGS) -c -o baro.o $(FIRM_DIR)/baro.cpp

sonars.o: $(FIRM_DIR)/sonars.cpp $(FIRM_DIR)/sonars.hpp
	@$(CC) $(CFLAGS) -c -o sonars.o $(FIRM_DIR)/sonars.cpp

rc.o: $(FIRM_DIR)/rc.cpp $(FIRM_DIR)/rc.hpp
	@$(CC) $(CFLAGS) -c -o rc.o $(FIRM_DIR)/rc.cpp

hover.o: $(FIRM_DIR)/hover.cpp $(FIRM_DIR)/hover.hpp $(FIRM_DIR)/rc.hpp
	@$(CC) $(CFLAGS) -c -o hover.o $(FIRM_DIR)/hover.cpp

board.o: board.cpp $(FIRM_DIR)/board.hpp
	@$(CC) $(CFLAGS) -I$(FIRM_DIR) -c -o board.o board.cpp

board_rx.o: board_rx.cpp $(FIRM_DIR)/board.hpp
	@$(CC) $(CFLAGS) -I$(FIRM_DIR) -c -o board_rx.o board_rx.cpp

clean:
	rm -rf *.o obj $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)

PRE_FLASH = stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
DO_FLASH  = stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

flash: flash_$(TARGET)

flash_$(TARGET): $(TARGET_HEX)
	$(PRE_FLASH)	
	sleep 1
	echo -n 'R' >$(SERIAL_DEVICE)
	$(DO_FLASH)

unbrick: unbrick_$(TARGET)

unbrick_$(TARGET): $(TARGET_HEX)
	$(PRE_FLASH)
	$(DO_FLASH)

commit:
	git commit -a --allow-empty-message -m ''
	git push

debug:
	miniterm.py $(SERIAL_DEVICE) 115200

listen:
	miniterm.py $(SERIAL_DEVICE) 115200
