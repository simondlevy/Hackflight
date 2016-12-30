# Makefile for BreezySTM32F3 examples
#
# Copyright (C) 2016 Simon D. Levy
#
# This file is part of BreezySTM32.
#
# BreezySTM32 is free software: you can redistribute it and/or modify
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

TARGET    ?= hackflight

BREEZY_DIR ?= $(HOME)/Desktop/BreezySTM32

F3_DIR = $(BREEZY_DIR)/f3
DRIVERS_DIR = $(F3_DIR)/drivers

# Compile-time options
OPTIONS   ?=

# Debugger optons, must be empty or GDB
DEBUG     ?=

# Insert the debugging hardfault debugger
# releases should not be built with this flag as it does not disable pwm output
DEBUG_HARDFAULTS ?=

# Serial port/Device for flashing
SERIAL_DEVICE   ?= /dev/ttyACM0

## V                 : Set verbosity level based on the V= parameter
##                     V=0 Low
##                     V=1 High
export AT := @

ifndef V
export V1    := $(AT)
export STDOUT   :=
else ifeq ($(V), 0)
export V1    := $(AT)
export STDOUT:= "> /dev/null"
export MAKE  := $(MAKE) --no-print-directory
else ifeq ($(V), 1)
export V1    :=
export STDOUT   :=
endif

###############################################################################
# Things that need to be maintained as the source changes
#

# Working directories
ROOT            := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
SRC_DIR         = $(ROOT)
OBJECT_DIR      = $(ROOT)/obj
BIN_DIR         = $(ROOT)/obj
CMSIS_DIR       = $(BREEZY_DIR)/lib/CMSIS

FEATURES    = VCP

ifeq ($(DEBUG_HARDFAULTS),F7)
CFLAGS               += -DDEBUG_HARDFAULTS
endif

REVISION = $(shell git log -1 --format="%h")

# Search path for sources
VPATH           := $(SRC_DIR)
USBFS_DIR       = $(BREEZY_DIR)/lib/STM32_USB-FS-Device_Driver
USBPERIPH_SRC   = $(notdir $(wildcard $(USBFS_DIR)/src/*.c))
FATFS_DIR       = $(ROOT)/lib/FatFS
FATFS_SRC       = $(notdir $(wildcard $(FATFS_DIR)/*.c))

CSOURCES        := $(shell find $(SRC_DIR) -name '*.c')

STDPERIPH_DIR   = $(BREEZY_DIR)/lib/STM32F30x_StdPeriph_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
EXCLUDES        = stm32f30x_crc.c \
                  stm32f30x_can.c

STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))
DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

VPATH           := $(VPATH):$(CMSIS_DIR)/CM1/CoreSupport:$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x
CMSIS_SRC       = $(notdir $(wildcard $(CMSIS_DIR)/CM1/CoreSupport/*.c \
                  $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x/*.c))

INCLUDE_DIRS    := $(SRC_DIR) \
                  $(F3_DIR) \
                  $(STDPERIPH_DIR)/inc \
                  $(CMSIS_DIR)/CM1/CoreSupport \
                  $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x

ifneq ($(filter VCP, $(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(USBFS_DIR)/inc 

VPATH           := $(VPATH):$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC)\
                        $(USBPERIPH_SRC)
endif

LD_SCRIPT       = $(F3_DIR)/stm32_flash_f303_256k.ld

ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS    = -DSTM32F303xC -DSTM32F303

.DEFAULT_GOAL := bin

INCLUDE_DIRS    := $(INCLUDE_DIRS) $(TARGET_DIR)

VPATH           := $(VPATH):$(TARGET_DIR)

HACKFLIGHT_DIR = /home/levys/Desktop/hackflight
FIRM_DIR = $(HACKFLIGHT_DIR)/firmware

HACKFLIGHT_SRC = $(FIRM_DIR)/imu.cpp

BREEZY_SRC = \
            $(F3_DIR)/adc.c \
            $(F3_DIR)/buf_writer.c \
            $(F3_DIR)/spi.c \
            $(F3_DIR)/exti.c \
            $(F3_DIR)/io.c \
            $(F3_DIR)/light_led.c \
            $(F3_DIR)/main.c \
            $(F3_DIR)/rcc.c \
            $(F3_DIR)/serial.c \
            $(F3_DIR)/serial_uart.c \
            $(F3_DIR)/system.c \
			$(F3_DIR)/target.c \
            $(F3_DIR)/timer.c \
            $(F3_DIR)/drivers/mpu.c \
            $(F3_DIR)/hw_config.c \
            $(F3_DIR)/serial_usb_vcp.c \
            $(F3_DIR)/stm32_it.c \
            $(F3_DIR)/usb_desc.c \
            $(F3_DIR)/usb_endp.c \
            $(F3_DIR)/usb_io.c \
            $(F3_DIR)/usb_istr.c \
            $(F3_DIR)/usb_prop.c \
            $(F3_DIR)/usb_pwr.c \
            $(F3_DIR)/adc_stm32f30x.c \
            $(F3_DIR)/i2c_stm32f30x.c \
            $(F3_DIR)/dma.c \
            $(F3_DIR)/gpio_stm32f30x.c \
            $(F3_DIR)/serial_uart_stm32f30x.c \
            $(F3_DIR)/startup_stm32f30x_md_gcc.S \
            $(F3_DIR)/system_stm32f30x.c \
            $(F3_DIR)/timer_stm32f30x.c \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC)

BOARD_SRC := tmp.cpp

TARGET_SRC := $(HACKFLIGHT_SRC) $(BREEZY_SRC) $(BOARD_SRC)

# Search path and source files for the ST stdperiph library
VPATH        := $(VPATH):$(STDPERIPH_DIR)/src

###############################################################################
# Things that might need changing to use different tools
#

# Find out if ccache is installed on the system
CCACHE := ccache
RESULT = $(shell (which $(CCACHE) > /dev/null 2>&1; echo $$?) )
ifneq ($(RESULT),0)
CCACHE :=
endif

ARM_SDK_PREFIX ?= arm-none-eabi-

# Tool names
CROSS_CC    := $(CCACHE) $(ARM_SDK_PREFIX)gcc
CROSS_CXX   := $(CCACHE) $(ARM_SDK_PREFIX)g++
OBJCOPY     := $(ARM_SDK_PREFIX)objcopy
SIZE        := $(ARM_SDK_PREFIX)size

#
# Tool options.
#

ifeq ($(DEBUG),GDB)
OPTIMIZE    = -O0
LTO_FLAGS   = $(OPTIMIZE)
else
ifeq ($(TARGET),$(filter $(TARGET),$(F1_TARGETS)))
OPTIMIZE    = -Os
else
OPTIMIZE    = -Ofast
endif
LTO_FLAGS   = -flto -fuse-linker-plugin $(OPTIMIZE)
endif

DEBUG_FLAGS = -ggdb3 -DDEBUG

CFLAGS      += $(ARCH_FLAGS) \
              $(LTO_FLAGS) \
              $(addprefix -D,$(OPTIONS)) \
              $(addprefix -I,$(INCLUDE_DIRS)) \
              $(DEBUG_FLAGS) \
              -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion \
              -ffunction-sections \
              -fdata-sections \
              -pedantic \
              $(DEVICE_FLAGS) \
              -DUSE_STDPERIPH_DRIVER \
              -D$(TARGET) \
              $(TARGET_FLAGS) \
              -D'__TARGET__="$(TARGET)"' \
              -D'__REVISION__="$(REVISION)"' \
              -save-temps=obj \
              -MMD -MP

ASFLAGS     = $(ARCH_FLAGS) \
              -x assembler-with-cpp \
              $(addprefix -I,$(INCLUDE_DIRS)) \
              -MMD -MP

LDFLAGS     = -lm \
              -nostartfiles \
              --specs=nano.specs \
              -lc \
              -lnosys \
              $(ARCH_FLAGS) \
              $(LTO_FLAGS) \
              $(DEBUG_FLAGS) \
              -static \
              -Wl,-gc-sections,-Map,$(TARGET_MAP) \
              -Wl,--cref \
              -Wl,--no-wchar-size-warning \
              -T$(LD_SCRIPT)

###############################################################################
# No user-serviceable parts below
###############################################################################

CPPCHECK        = cppcheck $(CSOURCES) --enable=all --platform=unix64 \
                  --std=c99 --inline-suppr --quiet --force \
                  $(addprefix -I,$(INCLUDE_DIRS)) \
                  -I/usr/include -I/usr/include/linux

#
# Things we will build
#
TARGET_BIN      = $(BIN_DIR)/$(TARGET).bin
TARGET_ELF      = $(OBJECT_DIR)/$(TARGET).elf
TARGET_OBJS     = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(TARGET_SRC))))
TARGET_DEPS     = $(addsuffix .d,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(TARGET_SRC))))
TARGET_MAP      = $(OBJECT_DIR)/$(TARGET).map


CLEAN_ARTIFACTS := $(TARGET_BIN)
CLEAN_ARTIFACTS += $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)

# Make sure build date and revision is updated on every incremental build
$(OBJECT_DIR)/$(TARGET)/build/version.o : $(TARGET_SRC)

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_BIN): $(TARGET_ELF)
	$(OBJCOPY) -O binary $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(V1) echo Linking $(TARGET)
	$(V1) $(CROSS_CC) -o $@ $^ $(LDFLAGS)
	 $(SIZE) $(TARGET_ELF)

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) echo "%% $(notdir $<)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(CFLAGS) $<

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.cpp
	$(V1) mkdir -p $(dir $@)
	$(V1) echo "%% $(notdir $<)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	$(V1) mkdir -p $(dir $@)
	$(V1) echo "%% $(notdir $<)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	$(V1) mkdir -p $(dir $@)
	$(V1) echo "%% $(notdir $<)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<

## sample            : Build all sample (travis) targets
sample: $(SAMPLE_TARGETS)

## clean             : clean up temporary / machine-generated files
clean:
	 echo "Cleaning $(TARGET)"
	 rm -f $(CLEAN_ARTIFACTS)
	 rm -rf $(OBJECT_DIR)
	 echo "Cleaning $(TARGET) succeeded."

## clean_<TARGET>    : clean up one specific target
$(CLEAN_TARGETS) :
	 $(MAKE) -j TARGET=$(subst clean_,,$@) clean

## <TARGET>_clean    : clean up one specific target (alias for above)
$(TARGETS_CLEAN) :
	 $(MAKE) -j TARGET=$(subst _clean,,$@) clean

## clean_all         : clean all valid targets
clean_all:$(CLEAN_TARGETS)

## all_clean         : clean all valid targets (alias for above)
all_clean:$(TARGETS_CLEAN)

PRE_FLASH = echo -n 'R' >$(SERIAL_DEVICE)

POST_FLASH = dfu-util -a 0 -s 0x08000000:leave -D $(TARGET_BIN)

preflash:
	$(PRE_FLASH)

postflash:
	$(POST_FLASH)

flash_$(TARGET): $(TARGET_BIN)
	$(PRE_FLASH)
	sleep 4
	$(UNBRICK):

## flash             : flash firmware (.hex) onto flight controller
flash: flash_$(TARGET)

bin:
	 $(MAKE) -j $(TARGET_BIN)

unbrick_$(TARGET): $(TARGET_BIN)
	$(POST_FLASH)

## unbrick           : unbrick flight controller
unbrick: unbrick_$(TARGET)

## cppcheck          : run static analysis on C source code
cppcheck: $(CSOURCES)
	 $(CPPCHECK)

cppcheck-result.xml: $(CSOURCES)
	 $(CPPCHECK) --xml-version=2 2> cppcheck-result.xml

# rebuild everything when makefile changes
$(TARGET_OBJS) : Makefile

# include auto-generated dependencies
-include $(TARGET_DEPS)

listen:
	miniterm.py $(SERIAL_DEVICE) 115200
