##############################################################################
#   Makefile for AlienflightF3V1 board
#
#   Copyright (c) 2018 Simon D. Levy
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
##############################################################################

# Things that the user might override on the commandline

# The target to build
TARGET    ?= ALIENFLIGHTF3

# Where you put Cleanflight
CF_DIR := $(HOME)/Desktop/cleanflight

# Where you put Arduino libraries
ARDUINO_DIR     := $(HOME)/Documents/Arduino/libraries

# Compile-time options
OPTIONS   ?=

# compile for OpenPilot BootLoader support
OPBL      ?= no

# Debugger optons:
#   empty           - ordinary build with all optimizations enabled
#   RELWITHDEBINFO  - ordinary build with debug symbols and all optimizations enabled
#   GDB             - debug build with minimum number of optimizations
DEBUG     ?=

# Insert the debugging hardfault debugger
# releases should not be built with this flag as it does not disable pwm output
DEBUG_HARDFAULTS ?=

# Serial port/Device for flashing
SERIAL_DEVICE   ?= /dev/ttyACM0

# Flash size (KB).  Some low-end chips actually have more flash than advertised, use this to override.
FLASH_SIZE ?=

##############################################################################
## V                 : Set verbosity level based on the V= parameter
##                     V=0 Low
##                     V=1 High
export AT := @

ifndef V
export V0    :=
export V1    := $(AT)
export STDOUT   :=
else ifeq ($(V), 0)
export V0    := $(AT)
export V1    := $(AT)
export STDOUT:= "> /dev/null"
export MAKE  := $(MAKE) --no-print-directory
else ifeq ($(V), 1)
export V0    :=
export V1    :=
export STDOUT   :=
endif

###############################################################################
# Things that need to be maintained as the source changes
#

FORKNAME      = hackflight

# Working directories
ROOT            := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
SRC_DIR         := $(CF_DIR)/src/main
OBJECT_DIR      := $(ROOT)/obj/main
BIN_DIR         := $(ROOT)/obj
LIB_DIR         := $(CF_DIR)/lib/main
CMSIS_DIR       := $(LIB_DIR)/CMSIS
HF_DIR          := $(ARDUINO_DIR)/Hackflight/src
IMU_DIR         := $(ARDUINO_DIR)/MPU/src
RX_DIR          := $(ARDUINO_DIR)/SpektrumDSM/src
I2C_DIR         := $(ARDUINO_DIR)/CrossPlatformDataBus/src
INCLUDE_DIRS    := $(SRC_DIR) $(SRC_DIR)/target $(HF_DIR) $(IMU_DIR) $(I2C_DIR) $(RX_DIR)
LINKER_DIR      := $(SRC_DIR)/target/link

# configure some variables dependent upon what type of system this is

OSFAMILY := linux
LINUX := 1

# configure some directories that are relative to wherever CF_DIR is located
ifndef TOOLS_DIR
TOOLS_DIR := $(CF_DIR)/tools
endif
BUILD_DIR := $(ROOT)/build
DL_DIR    := $(ROOT)/downloads

export RM := rm

ARM_SDK_PREFIX := arm-none-eabi-

# default xtal value for F4 targets
HSE_VALUE       ?= 8000000

# used for turning on features like VCP and SDCARD
FEATURES        = VCP

OFFICIAL_TARGETS  = ALIENFLIGHTF3 ALIENFLIGHTF4 ANYFCF7 BETAFLIGHTF3 BLUEJAYF4 FURYF4 REVO SIRINFPV SPARKY SPRACINGF3 SPRACINGF3EVO SPRACINGF3NEO SPRACINGF3MINI SPRACINGF4EVO SPRACINGF7DUAL STM32F3DISCOVERY
ALT_TARGETS       = $(sort $(filter-out target, $(basename $(notdir $(wildcard $(SRC_DIR)/target/*/*.mk)))))
NOBUILD_TARGETS   = $(sort $(filter-out target, $(basename $(notdir $(wildcard $(SRC_DIR)/target/*/*.nomk)))))
OPBL_TARGETS      = $(filter %_OPBL, $(ALT_TARGETS))
OSD_SLAVE_TARGETS = SPRACINGF3OSD

ifeq ($(filter $(TARGET),$(NOBUILD_TARGETS)), $(TARGET))
ALTERNATES    := $(sort $(filter-out target, $(basename $(notdir $(wildcard $(SRC_DIR)/target/$(TARGET)/*.mk)))))
$(error The target specified, $(TARGET), cannot be built. Use one of the ALT targets: $(ALTERNATES))
endif

UNSUPPORTED_TARGETS := \
	AFROMINI \
	ALIENFLIGHTF1 \
	BEEBRAIN \
	CC3D \
	CC3D_OPBL \
	CJMCU \
	MICROSCISKY \
	NAZE

SUPPORTED_TARGETS := $(filter-out $(UNSUPPORTED_TARGETS), $(VALID_TARGETS))


ifeq ($(filter $(TARGET),$(ALT_TARGETS)), $(TARGET))
BASE_TARGET    := $(firstword $(subst /,, $(subst ./src/target/,, $(dir $(wildcard $(SRC_DIR)/target/*/$(TARGET).mk)))))
include $(SRC_DIR)/target/$(BASE_TARGET)/$(TARGET).mk
else
BASE_TARGET    := $(TARGET)
endif

ifeq ($(filter $(TARGET),$(OPBL_TARGETS)), $(TARGET))
OPBL            = yes
endif

ifeq ($(filter $(TARGET),$(OSD_SLAVE_TARGETS)), $(TARGET))
# build an OSD SLAVE
OSD_SLAVE       = yes
else
# build an FC
FC              = yes
endif

# silently ignore if the file is not present. Allows for target specific.
F3_TARGETS  += $(TARGET)

F4_TARGETS      := $(F405_TARGETS) $(F411_TARGETS) $(F446_TARGETS)
F7_TARGETS      := $(F7X2RE_TARGETS) $(F7X5XE_TARGETS) $(F7X5XG_TARGETS) $(F7X5XI_TARGETS) $(F7X6XG_TARGETS)

ifeq ($(TARGET),$(filter $(TARGET),$(F3_TARGETS)))
TARGET_MCU := STM32F3

else ifeq ($(TARGET),$(filter $(TARGET), $(F4_TARGETS)))
TARGET_MCU := STM32F4

else ifeq ($(TARGET),$(filter $(TARGET), $(F7_TARGETS)))
TARGET_MCU := STM32F7

else ifeq ($(TARGET),$(filter $(TARGET), $(SITL_TARGETS)))
TARGET_MCU := SITL
SIMULATOR_BUILD = yes

else ifeq ($(TARGET),$(filter $(TARGET), $(F1_TARGETS)))
TARGET_MCU := STM32F1
else
$(error Unknown target MCU specified.)
endif

ifneq ($(BASE_TARGET), $(TARGET))
TARGET_FLAGS  	:= $(TARGET_FLAGS) -D$(BASE_TARGET)
endif

TARGET_FLAGS  	:= $(TARGET_FLAGS) -D$(TARGET_MCU)

REVISION := $(shell git log -1 --format="%h")

FC_VER_MAJOR := $(shell grep " FC_VERSION_MAJOR" $(SRC_DIR)/build/version.h | awk '{print $$3}' )
FC_VER_MINOR := $(shell grep " FC_VERSION_MINOR" $(SRC_DIR)/build/version.h | awk '{print $$3}' )
FC_VER_PATCH := $(shell grep " FC_VERSION_PATCH" $(SRC_DIR)/build/version.h | awk '{print $$3}' )

FC_VER := $(FC_VER_MAJOR).$(FC_VER_MINOR).$(FC_VER_PATCH)

# Search path for sources
VPATH           := $(SRC_DIR):$(SRC_DIR)/startup
USBFS_DIR       = $(LIB_DIR)/STM32_USB-FS-Device_Driver
USBPERIPH_SRC   = $(notdir $(wildcard $(USBFS_DIR)/src/*.c))
FATFS_DIR       = $(LIB_DIR)/FatFS
FATFS_SRC       = $(notdir $(wildcard $(FATFS_DIR)/*.c))

CSOURCES        := $(shell find $(SRC_DIR) -name '*.c')

LD_FLAGS         :=

#
# Default Tool options - can be overridden in {mcu}.mk files.
#
ifeq ($(DEBUG),GDB)
OPTIMISE_DEFAULT      := -Og

LTO_FLAGS             := $(OPTIMISE_DEFAULT)
DEBUG_FLAGS            = -ggdb3 -DDEBUG
else
ifeq ($(DEBUG),INFO)
DEBUG_FLAGS            = -ggdb3
endif
OPTIMISATION_BASE     := -flto -fuse-linker-plugin -ffast-math
OPTIMISE_DEFAULT      := -O2
OPTIMISE_SPEED        := -Ofast
OPTIMISE_SIZE         := -Os

LTO_FLAGS             := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
endif

VPATH 			:= $(VPATH):$(CF_DIR)/make/mcu
VPATH 			:= $(VPATH):$(CF_DIR)/make

# start specific includes
ifeq ($(OPBL),yes)
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k_opbl.ld
endif

TARGET_FLASH   := 256

STDPERIPH_DIR   = $(LIB_DIR)/STM32F3/Drivers/STM32F30x_StdPeriph_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
EXCLUDES        = stm32f30x_crc.c \
                  stm32f30x_can.c
STARTUP_SRC     = startup_stm32f30x_md_gcc.S

STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))
DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)

VPATH           := $(VPATH):$(CMSIS_DIR)/Core/Include
CMSIS_SRC       = $(notdir $(wildcard $(LIB_DIR)/STM32F3/Drivers/CMSIS/Device/ST/STM32F30x/*.c))

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/inc \
                   $(CMSIS_DIR)/Core/Include \
                   $(LIB_DIR)/STM32F3/Drivers/CMSIS/Device/ST/STM32F30x

ifneq ($(filter VCP, $(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(USBFS_DIR)/inc \
                   $(SRC_DIR)/vcp

VPATH           := $(VPATH):$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC)\
                        $(USBPERIPH_SRC)
endif

ifneq ($(filter SDCARD, $(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(FATFS_DIR) \

VPATH           := $(VPATH):$(FATFS_DIR)
endif

LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k.ld

ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS    = -DSTM32F303xC -DSTM32F303

VCP_SRC = \
            vcp/hw_config.c \
            vcp/stm32_it.c \
            vcp/usb_desc.c \
            vcp/usb_endp.c \
            vcp/usb_istr.c \
            vcp/usb_prop.c \
            vcp/usb_pwr.c \
            drivers/serial_usb_vcp.c \
            drivers/usb_io.c


MCU_COMMON_SRC = \
        target/system_stm32f30x.c \
        drivers/adc_stm32f30x.c \
        drivers/bus_spi_stdperiph.c \
        drivers/dma.c \
        drivers/pwm_output_dshot.c \
        drivers/system_stm32f30x.c \
        drivers/timer_stm32f30x.c \
        drivers/light_led.c \
        drivers/serial_uart_stm32f30x.c \
	    drivers/serial_uart_pinconfig.c \
	    drivers/serial_pinconfig.c \
	    drivers/serial_uart_init.c \
	    drivers/serial_uart.c \
	    drivers/serial.c \
	    drivers/bus_i2c_busdev.c \
	    drivers/bus_i2c_config.c \
        drivers/bus_i2c_stm32f30x.c \
	    io/serial.c \
	    pg/pg.c \
	    pg/bus_i2c.c

DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4

# Configure default flash sizes for the targets (largest size specified gets hit first) if flash not specified already.
ifeq ($(FLASH_SIZE),)
ifneq ($(TARGET_FLASH),)
FLASH_SIZE := $(TARGET_FLASH)
else
$(error FLASH_SIZE not configured for target $(TARGET))
endif
endif

DEVICE_FLAGS  := $(DEVICE_FLAGS) -DFLASH_SIZE=$(FLASH_SIZE)

ifneq ($(HSE_VALUE),)
DEVICE_FLAGS  := $(DEVICE_FLAGS) -DHSE_VALUE=$(HSE_VALUE)
endif

TARGET_DIR     = $(SRC_DIR)/target/$(BASE_TARGET)
TARGET_DIR_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))

ifeq ($(OPBL),yes)
TARGET_FLAGS := -DOPBL $(TARGET_FLAGS)
.DEFAULT_GOAL := binary
else
.DEFAULT_GOAL := hex
endif

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(LIB_DIR)/MAVLink

INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(TARGET_DIR)

VPATH           := $(VPATH):$(TARGET_DIR)

COMMON_SRC = \
            $(TARGET_DIR_SRC) \
            drivers/io.c \
            drivers/system.c \
            drivers/timer_common.c \
            drivers/timer.c \
            drivers/pwm_output.c \
            drivers/rcc.c 

COMMON_DEVICE_SRC = \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC)

ifeq ($(OSD_SLAVE),yes)
TARGET_FLAGS := -DUSE_OSD_SLAVE $(TARGET_FLAGS)
COMMON_SRC := $(COMMON_SRC) $(OSD_SLAVE_SRC) $(COMMON_DEVICE_SRC)
else
COMMON_SRC := $(COMMON_SRC) $(COMMON_DEVICE_SRC)
endif

ifeq ($(SIMULATOR_BUILD),yes)
TARGET_FLAGS := -DSIMULATOR_BUILD $(TARGET_FLAGS)
endif

SPEED_OPTIMISED_SRC := ""
SIZE_OPTIMISED_SRC  := ""

SRC := $(STARTUP_SRC) $(MCU_COMMON_SRC) $(TARGET_SRC) $(VARIANT_SRC)

SRC += $(COMMON_SRC)

#excludes
SRC   := $(filter-out $(MCU_EXCLUDES), $(SRC))

ifneq ($(filter VCP,$(FEATURES)),)
SRC += $(VCP_SRC)
endif

ifneq ($(filter MSC,$(FEATURES)),)
SRC += $(MSC_SRC)
endif
# end target specific make file checks

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

# Tool names
CROSS_CC    := $(CCACHE) $(ARM_SDK_PREFIX)gcc
CROSS_CXX   := $(CCACHE) $(ARM_SDK_PREFIX)g++
CROSS_GDB   := $(ARM_SDK_PREFIX)gdb
OBJCOPY     := $(ARM_SDK_PREFIX)objcopy
OBJDUMP     := $(ARM_SDK_PREFIX)objdump
SIZE        := $(ARM_SDK_PREFIX)size

#
# Tool options.
#
CC_DEBUG_OPTIMISATION   := $(OPTIMISE_DEFAULT)
CC_DEFAULT_OPTIMISATION := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
CC_SPEED_OPTIMISATION   := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
CC_SIZE_OPTIMISATION    := $(OPTIMISATION_BASE) $(OPTIMISE_SIZE)

CFLAGS     += $(ARCH_FLAGS) \
              $(addprefix -D,$(OPTIONS)) \
              $(addprefix -I,$(INCLUDE_DIRS)) \
              $(DEBUG_FLAGS) \
              -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion \
              -ffunction-sections \
              -fdata-sections \
              -pedantic \
              $(DEVICE_FLAGS) \
              -D_GNU_SOURCE \
              -DUSE_STDPERIPH_DRIVER \
              -D$(TARGET) \
              $(TARGET_FLAGS) \
              -D'__FORKNAME__="$(FORKNAME)"' \
              -D'__TARGET__="$(TARGET)"' \
              -D'__REVISION__="$(REVISION)"' \
              -save-temps=obj \
              -MMD -MP \
              $(EXTRA_FLAGS)

ASFLAGS     = $(ARCH_FLAGS) \
              -x assembler-with-cpp \
              $(addprefix -I,$(INCLUDE_DIRS)) \
              -MMD -MP

ifeq ($(LD_FLAGS),)
LD_FLAGS     = -lm \
              -nostartfiles \
              --specs=nano.specs \
              -lc \
              -lnosys \
              $(ARCH_FLAGS) \
              $(LTO_FLAGS) \
              $(DEBUG_FLAGS) \
              -static \
              -Wl,-gc-sections,-Map,$(TARGET_MAP) \
              -Wl,-L$(LINKER_DIR) \
              -Wl,--cref \
              -Wl,--no-wchar-size-warning \
              -Wl,--print-memory-usage \
              -T$(LD_SCRIPT)
endif

# Hackflight support
BOARD := alienflightf3v1
BOARD_DIR := ../../boards/$(BOARD)
INCLUDE_DIRS += $(BOARD_DIR)
IMU_SRC := $(IMU_DIR)/MPU.cpp $(IMU_DIR)/MPU6xx0.cpp $(IMU_DIR)/MPU6050.cpp
RX_SRC := $(RX_DIR)/SpektrumDSM.cpp
HF_SRC := ../../common/main.c ../../common/i2c.cpp ../../common/dsmx.cpp ../../common/motors.cpp 
BOARD_SRC := $(BOARD_DIR)/$(BOARD).cpp  
SKETCH_SRC := ./alienflightf3v1_dsmx.cpp
SRC += $(HF_SRC) $(IMU_SRC) $(RX_SRC) $(BOARD_SRC) $(SKETCH_SRC)

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
TARGET_BIN      = $(BIN_DIR)/$(FORKNAME)_$(FC_VER)_$(TARGET).bin
TARGET_HEX      = $(BIN_DIR)/$(FORKNAME)_$(FC_VER)_$(TARGET).hex
TARGET_ELF      = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).elf
TARGET_LST      = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).lst
TARGET_OBJS     = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(SRC))))
TARGET_DEPS     = $(addsuffix .d,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(SRC))))
TARGET_MAP      = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map

CLEAN_ARTIFACTS := $(TARGET_BIN)
CLEAN_ARTIFACTS += $(TARGET_HEX)
CLEAN_ARTIFACTS += $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)
CLEAN_ARTIFACTS += $(TARGET_LST)

# Make sure build date and revision is updated on every incremental build
$(OBJECT_DIR)/$(TARGET)/build/version.o : $(SRC)

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_LST): $(TARGET_ELF)
	$(V0) $(OBJDUMP) -S --disassemble $< > $@

$(TARGET_HEX): $(TARGET_ELF)
	@echo "Creating HEX $(TARGET_HEX)" "$(STDOUT)"
	$(V1) $(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_BIN): $(TARGET_ELF)
	@echo "Creating BIN $(TARGET_BIN)" "$(STDOUT)"
	$(V1) $(OBJCOPY) -O binary $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	@echo "Linking: $(TARGET)" "$(STDOUT)"
	$(V1) $(CROSS_CXX) -o $@ $^ $(LD_FLAGS)
	$(V1) $(SIZE) $(TARGET_ELF)

# Compile .c
ifeq ($(DEBUG),GDB)
$(OBJECT_DIR)/$(TARGET)/%.o: %.c 
	$(V1) mkdir -p $(dir $@)
	$(V1) echo "%% (debug) $<" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(CC_DEBUG_OPTIMISATION) $<
else
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) $(if $(findstring $(subst ./src/,,$<),$(SPEED_OPTIMISED_SRC)), \
	echo "%% (speed optimised) $<" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(CC_SPEED_OPTIMISATION) $<, \
	$(if $(findstring $(subst ./src/,,$<),$(SIZE_OPTIMISED_SRC)), \
	echo "%% (size optimised) $<" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(CC_SIZE_OPTIMISATION) $<, \
	echo "%% $<" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(CC_DEFAULT_OPTIMISATION) $<))
endif

# Compile .cpp
ifeq ($(DEBUG),GDB)
$(OBJECT_DIR)/$(TARGET)/%.o: %.cpp
	$(V1) mkdir -p $(dir $@)
	$(V1) echo "%% (debug) $<" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(CC_DEBUG_OPTIMISATION) $<
else
$(OBJECT_DIR)/$(TARGET)/%.o: %.cpp
	$(V1) mkdir -p $(dir $@)
	$(V1) $(if $(findstring $(subst ./src/,,$<),$(SPEED_OPTIMISED_SRC)), \
	echo "%% (speed optimised) $<" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(CC_SPEED_OPTIMISATION) $<, \
	$(if $(findstring $(subst ./src/,,$<),$(SIZE_OPTIMISED_SRC)), \
	echo "%% (size optimised) $<" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(CC_SIZE_OPTIMISATION) $<, \
	echo "%% $<" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(CC_DEFAULT_OPTIMISATION) $<))
endif

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	$(V1) mkdir -p $(dir $@)
	@echo "%% $<" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	$(V1) mkdir -p $(dir $@)
	@echo "%% $<" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<


## all               : Build all targets (excluding unsupported)
all: $(SUPPORTED_TARGETS)

## all_with_unsupported : Build all targets (including unsupported)
all_with_unsupported: $(VALID_TARGETS)

## unsupported : Build unsupported targets
unsupported: $(UNSUPPORTED_TARGETS)

## official          : Build all official (travis) targets
official: $(OFFICIAL_TARGETS)

## targets-group-1   : build some targets

$(VALID_TARGETS):
	$(V0) @echo "Building $@" && \
	$(MAKE) binary hex TARGET=$@ && \
	echo "Building $@ succeeded."

$(NOBUILD_TARGETS):
	$(MAKE) TARGET=$@

CLEAN_TARGETS = $(addprefix clean_,$(VALID_TARGETS) )
TARGETS_CLEAN = $(addsuffix _clean,$(VALID_TARGETS) )

## clean             : clean up temporary / machine-generated files
clean:
	rm -rf obj lib

## clean_test        : clean up temporary / machine-generated files (tests)
clean_test:
	$(V0) cd src/test && $(MAKE) clean || true

## clean_<TARGET>    : clean up one specific target
$(CLEAN_TARGETS):
	$(V0) $(MAKE) -j TARGET=$(subst clean_,,$@) clean

## <TARGET>_clean    : clean up one specific target (alias for above)
$(TARGETS_CLEAN):
	$(V0) $(MAKE) -j TARGET=$(subst _clean,,$@) clean

## clean_all         : clean all valid targets
clean_all: $(CLEAN_TARGETS)

## all_clean         : clean all valid targets (alias for above)
all_clean: $(TARGETS_CLEAN)

flash: flash_$(TARGET)

flash_$(TARGET): $(TARGET_BIN)
	$(V0) echo -n 'R' >$(SERIAL_DEVICE)
	sleep 2
	dfu-util -a 0 -s 0x08000000:leave -D $(TARGET_BIN)

## flash             : flash firmware (.hex) onto flight controller
st-flash_$(TARGET): $(TARGET_BIN)
	$(V0) st-flash --reset write $< 0x08000000

## st-flash          : flash firmware (.bin) onto flight controller
st-flash: st-flash_$(TARGET)

ifneq ($(OPENOCD_COMMAND),)
openocd-gdb: $(TARGET_ELF)
	$(V0) $(OPENOCD_COMMAND) & $(CROSS_GDB) $(TARGET_ELF) -ex "target remote localhost:3333" -ex "load"
endif

binary:
	$(V0) $(MAKE) -j $(TARGET_BIN)

hex:
	$(V0) $(MAKE) -j $(TARGET_HEX)

unbrick_$(TARGET): $(TARGET_BIN)
	dfu-util -a 0 -s 0x08000000:leave -D $(TARGET_BIN)

## unbrick           : unbrick flight controller
unbrick: unbrick_$(TARGET)

## cppcheck          : run static analysis on C source code
cppcheck: $(CSOURCES)
	$(V0) $(CPPCHECK)

cppcheck-result.xml: $(CSOURCES)
	$(V0) $(CPPCHECK) --xml-version=2 2> cppcheck-result.xml

# mkdirs
$(DL_DIR):
	mkdir -p $@

$(TOOLS_DIR):
	mkdir -p $@

$(BUILD_DIR):
	mkdir -p $@

## version           : print firmware version
version:
	@echo $(FC_VER)

## help              : print this help message and exit
help: Makefile make/tools.mk
	@echo ""
	@echo "Makefile for the $(FORKNAME) firmware"
	@echo ""
	@echo "Usage:"
	@echo "        make [V=<verbosity>] [TARGET=<target>] [OPTIONS=\"<options>\"]"
	@echo "Or:"
	@echo "        make <target> [V=<verbosity>] [OPTIONS=\"<options>\"]"
	@echo ""
	@echo "Valid TARGET values are: $(VALID_TARGETS)"
	@echo ""
	@sed -n 's/^## //p' $?


check-target-independence:
	$(V1) for test_target in $(VALID_TARGETS); do \
		FOUND=$$(grep -rE "\W$${test_target}\W?" src | grep -vE "(//)|(/\*).*\W$${test_target}\W?" | grep -vE "^src/target"); \
		if [ "$${FOUND}" != "" ]; then \
			echo "Target dependencies found:"; \
			echo "$${FOUND}"; \
			exit 1; \
		fi; \
	done

check-fastram-usage-correctness:
	$(V1) NON_TRIVIALLY_INITIALIZED=$$(grep -Ern "\W?FAST_RAM_ZERO_INIT\W.*=.*" src/ | grep -Ev "=\s*(false|NULL|0(\.0*f?)?)\s*[,;]"); \
	if [ "$${NON_TRIVIALLY_INITIALIZED}" != "" ]; then \
		echo "Non-trivially initialized FAST_RAM_ZERO_INIT variables found, use FAST_RAM instead:"; \
		echo "$${NON_TRIVIALLY_INITIALIZED}"; \
		exit 1; \
	fi; \
	TRIVIALLY_INITIALIZED=$$(grep -Ern "\W?FAST_RAM\W.*;" src/ | grep -v "="); \
	EXPLICITLY_TRIVIALLY_INITIALIZED=$$(grep -Ern "\W?FAST_RAM\W.*;" src/ | grep -E "=\s*(false|NULL|0(\.0*f?)?)\s*[,;]"); \
	if [ "$${TRIVIALLY_INITIALIZED}$${EXPLICITLY_TRIVIALLY_INITIALIZED}" != "" ]; then \
		echo "Trivially initialized FAST_RAM variables found, use FAST_RAM_ZERO_INIT instead to save FLASH:"; \
		echo "$${TRIVIALLY_INITIALIZED}\n$${EXPLICITLY_TRIVIALLY_INITIALIZED}"; \
		exit 1; \
	fi;

# rebuild everything when makefile changes
$(TARGET_OBJS) : Makefile

# include auto-generated dependencies
-include $(TARGET_DEPS)

listen:
	miniterm.py $(SERIAL_DEVICE) 115200

commit:
	git commit -a -m="More AlienflightF3 Board implementation"; git push
