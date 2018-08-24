##############################################################################
#   Makefile for OpenPilot CC3D Revolution board
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
#

# The target to build, see VALID_TARGETS below
TARGET    ?= REVO

# Where you put Cleanflight
CF_DIR := $(HOME)/Desktop/cleanflight

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
SERIAL_DEVICE   ?= $(firstword $(wildcard /dev/ttyUSB*) no-port-found)

# Flash size (KB).  Some low-end chips actually have more flash than advertised, use this to override.
FLASH_SIZE ?=


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
INCLUDE_DIRS    := $(SRC_DIR) \
                   $(ROOT)/src/main/target
LINKER_DIR      := $(ROOT)/src/main/target/link

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

# Build tools, so we all share the same versions
# import macros common to all supported build systems
include $(CF_DIR)/make/system-id.mk

# developer preferences, edit these at will, they'll be gitignored
-include $(CF_DIR)/make/local.mk

# configure some directories that are relative to wherever ROOT_DIR is located
ifndef TOOLS_DIR
TOOLS_DIR := $(ROOT)/tools
endif
BUILD_DIR := $(ROOT)/build
DL_DIR    := $(ROOT)/downloads

export RM := rm

# import macros that are OS specific
include $(CF_DIR)/make/$(OSFAMILY).mk

# include the tools makefile
include $(CF_DIR)/make/tools.mk

# default xtal value for F4 targets
HSE_VALUE       ?= 8000000

# used for turning on features like VCP and SDCARD
FEATURES        =

include $(CF_DIR)/make/targets.mk

REVISION := $(shell git log -1 --format="%h")

FC_VER_MAJOR := $(shell grep " FC_VERSION_MAJOR" src/main/build/version.h | awk '{print $$3}' )
FC_VER_MINOR := $(shell grep " FC_VERSION_MINOR" src/main/build/version.h | awk '{print $$3}' )
FC_VER_PATCH := $(shell grep " FC_VERSION_PATCH" src/main/build/version.h | awk '{print $$3}' )

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

VPATH 			:= $(VPATH):$(ROOT)/make/mcu
VPATH 			:= $(VPATH):$(ROOT)/make

#
# F4 Make file include
#

ifeq ($(OPBL),yes)
ifeq ($(TARGET), $(filter $(TARGET),$(F405_TARGETS)))
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f405_opbl.ld
else ifeq ($(TARGET), $(filter $(TARGET),$(F411_TARGETS)))
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f411_opbl.ld
else
$(error No OPBL linker script specified for $(TARGET`))
endif
endif

#CMSIS
ifeq ($(PERIPH_DRIVER), HAL)
CMSIS_DIR      := $(ROOT)/lib/main/STM32F4/Drivers/CMSIS
STDPERIPH_DIR   = $(ROOT)/lib/main/STM32F4/Drivers/STM32F4xx_HAL_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/Src/*.c))
EXCLUDES        =
else
CMSIS_DIR      := $(ROOT)/lib/main/CMSIS
STDPERIPH_DIR   = $(ROOT)/lib/main/STM32F4/Drivers/STM32F4xx_StdPeriph_Driver
STDPERIPH_SRC   = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
EXCLUDES        = stm32f4xx_crc.c \
                  stm32f4xx_can.c \
                  stm32f4xx_fmc.c \
                  stm32f4xx_sai.c \
                  stm32f4xx_cec.c \
                  stm32f4xx_dsi.c \
                  stm32f4xx_flash_ramfunc.c \
                  stm32f4xx_fmpi2c.c \
                  stm32f4xx_lptim.c \
                  stm32f4xx_qspi.c \
                  stm32f4xx_spdifrx.c \
                  stm32f4xx_cryp.c \
                  stm32f4xx_cryp_aes.c \
                  stm32f4xx_hash_md5.c \
                  stm32f4xx_cryp_des.c \
                  stm32f4xx_rtc.c \
                  stm32f4xx_hash.c \
                  stm32f4xx_dbgmcu.c \
                  stm32f4xx_cryp_tdes.c \
                  stm32f4xx_hash_sha1.c
endif

ifeq ($(TARGET),$(filter $(TARGET), $(F411_TARGETS)))
EXCLUDES        += stm32f4xx_fsmc.c
TARGET_FLASH    := 512
else ifeq ($(TARGET),$(filter $(TARGET), $(F446_TARGETS)))
EXCLUDES        += stm32f4xx_fsmc.c
TARGET_FLASH    := 512
else
TARGET_FLASH    := 1024
endif

STDPERIPH_SRC   := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

ifeq ($(PERIPH_DRIVER), HAL)
#USB
USBCORE_DIR = $(ROOT)/lib/main/STM32F4/Middlewares/ST/STM32_USB_Device_Library/Core
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/Src/*.c))
EXCLUDES    = usbd_conf_template.c
USBCORE_SRC := $(filter-out ${EXCLUDES}, $(USBCORE_SRC))

USBCDC_DIR = $(ROOT)/lib/main/STM32F4/Middlewares/ST/STM32_USB_Device_Library/Class/CDC
USBCDC_SRC = $(notdir $(wildcard $(USBCDC_DIR)/Src/*.c))
EXCLUDES   = usbd_cdc_if_template.c
USBCDC_SRC := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))

VPATH := $(VPATH):$(USBCDC_DIR)/Src:$(USBCORE_DIR)/Src

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC)
else
USBCORE_DIR = $(ROOT)/lib/main/STM32_USB_Device_Library/Core
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/src/*.c))
USBOTG_DIR  = $(ROOT)/lib/main/STM32_USB_OTG_Driver
USBOTG_SRC  = $(notdir $(wildcard $(USBOTG_DIR)/src/*.c))
EXCLUDES    = usb_bsp_template.c \
              usb_conf_template.c \
              usb_hcd_int.c \
              usb_hcd.c \
              usb_otg.c

USBOTG_SRC  := $(filter-out ${EXCLUDES}, $(USBOTG_SRC))
USBCDC_DIR  = $(ROOT)/lib/main/STM32_USB_Device_Library/Class/cdc
USBCDC_SRC  = $(notdir $(wildcard $(USBCDC_DIR)/src/*.c))
EXCLUDES    = usbd_cdc_if_template.c
USBCDC_SRC  := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))
USBMSC_DIR  = $(ROOT)/lib/main/STM32_USB_Device_Library/Class/msc
USBMSC_SRC  = $(notdir $(wildcard $(USBMSC_DIR)/src/*.c))
EXCLUDES    = usbd_storage_template.c
USBMSC_SRC  := $(filter-out ${EXCLUDES}, $(USBMSC_SRC))
USBHID_DIR  = $(ROOT)/lib/main/STM32_USB_Device_Library/Class/hid
USBHID_SRC  = $(notdir $(wildcard $(USBHID_DIR)/src/*.c))
USBWRAPPER_DIR  = $(ROOT)/lib/main/STM32_USB_Device_Library/Class/hid_cdc_wrapper
USBWRAPPER_SRC  = $(notdir $(wildcard $(USBWRAPPER_DIR)/src/*.c))
VPATH       := $(VPATH):$(USBOTG_DIR)/src:$(USBCORE_DIR)/src:$(USBCDC_DIR)/src:$(USBMSC_DIR)/src:$(USBHID_DIR)/src:$(USBWRAPPER_DIR)/src

DEVICE_STDPERIPH_SRC := $(STDPERIPH_SRC) \
                        $(USBOTG_SRC) \
                        $(USBCORE_SRC) \
                        $(USBCDC_SRC) \
                        $(USBHID_SRC) \
                        $(USBWRAPPER_SRC) \
                        $(USBMSC_SRC)
endif

#CMSIS
VPATH           := $(VPATH):$(CMSIS_DIR)/Core/Include:$(ROOT)/lib/main/STM32F4/Drivers/CMSIS/Device/ST/STM32F4xx

ifeq ($(PERIPH_DRIVER), HAL)
CMSIS_SRC       :=
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/Inc \
                   $(USBCORE_DIR)/Inc \
                   $(USBCDC_DIR)/Inc \
                   $(CMSIS_DIR)/Include \
                   $(CMSIS_DIR)/Device/ST/STM32F4xx/Include \
                   $(ROOT)/src/main/vcp_hal
else
CMSIS_SRC       := $(notdir $(wildcard $(CMSIS_DIR)/CoreSupport/*.c \
                   $(ROOT)/lib/main/STM32F4/Drivers/CMSIS/Device/ST/STM32F4xx/*.c))
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(STDPERIPH_DIR)/inc \
                   $(USBOTG_DIR)/inc \
                   $(USBCORE_DIR)/inc \
                   $(USBCDC_DIR)/inc \
                   $(USBHID_DIR)/inc \
                   $(USBWRAPPER_DIR)/inc \
                   $(USBMSC_DIR)/inc \
                   $(USBFS_DIR)/inc \
                   $(CMSIS_DIR)/Core/Include \
                   $(ROOT)/lib/main/STM32F4/Drivers/CMSIS/Device/ST/STM32F4xx \
                   $(ROOT)/src/main/vcpf4
endif

ifneq ($(filter SDCARD,$(FEATURES)),)
INCLUDE_DIRS    := $(INCLUDE_DIRS) \
                   $(FATFS_DIR)
VPATH           := $(VPATH):$(FATFS_DIR)
endif

#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion

ifeq ($(TARGET),$(filter $(TARGET),$(F411_TARGETS)))
DEVICE_FLAGS    = -DSTM32F411xE
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f411.ld
STARTUP_SRC     = startup_stm32f411xe.s
else ifeq ($(TARGET),$(filter $(TARGET),$(F405_TARGETS)))
DEVICE_FLAGS    = -DSTM32F40_41xxx -DSTM32F405xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f405.ld
STARTUP_SRC     = startup_stm32f40xx.s
else ifeq ($(TARGET),$(filter $(TARGET),$(F446_TARGETS)))
DEVICE_FLAGS    = -DSTM32F446xx
LD_SCRIPT       = $(LINKER_DIR)/stm32_flash_f446.ld
STARTUP_SRC     = startup_stm32f446xx.s
else
$(error Unknown MCU for F4 target)
endif
DEVICE_FLAGS    += -DHSE_VALUE=$(HSE_VALUE)

MCU_COMMON_SRC = \
            target/system_stm32f4xx.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/adc_stm32f4xx.c \
            drivers/bus_i2c_stm32f10x.c \
            drivers/bus_spi_stdperiph.c \
            drivers/dma_stm32f4xx.c \
            drivers/inverter.c \
            drivers/light_ws2811strip_stdperiph.c \
            drivers/transponder_ir_io_stdperiph.c \
            drivers/pwm_output_dshot.c \
            drivers/serial_uart_init.c \
            drivers/serial_uart_stm32f4xx.c \
            drivers/system_stm32f4xx.c \
            drivers/timer_stm32f4xx.c

ifeq ($(PERIPH_DRIVER), HAL)
VCP_SRC = \
            vcp_hal/usbd_desc.c \
            vcp_hal/usbd_conf.c \
            vcp_hal/usbd_cdc_interface.c \
            drivers/serial_usb_vcp.c \
            drivers/usb_io.c
else
VCP_SRC = \
            vcpf4/stm32f4xx_it.c \
            vcpf4/usb_bsp.c \
            vcpf4/usbd_desc.c \
            vcpf4/usbd_usr.c \
            vcpf4/usbd_cdc_vcp.c \
            drivers/serial_usb_vcp.c \
            drivers/usb_io.c
endif

MSC_SRC = \
            drivers/usb_msc_f4xx.c \
            msc/usbd_msc_desc.c \
            msc/usbd_storage.c

ifneq ($(filter SDCARD,$(FEATURES)),)
MSC_SRC += \
            msc/usbd_storage_sd_spi.c
endif

ifneq ($(filter SDIO,$(FEATURES)),)
MSC_SRC += \
            msc/usbd_storage_sdio.c
MCU_COMMON_SRC += \
            drivers/sdio_f4xx.c
endif

ifneq ($(filter ONBOARDFLASH,$(FEATURES)),)
MSC_SRC += \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c
endif

DSP_LIB := $(ROOT)/lib/main/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4

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

TARGET_DIR     = $(ROOT)/src/main/target/$(BASE_TARGET)
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

include $(CF_DIR)/make/source.mk

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
              -std=gnu99 \
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
	@echo "Linking $(TARGET)" "$(STDOUT)"
ifeq ($(TARGET_MCU),STM32F1)
	@echo "Notice: STM32F1 based targets will be unsupported soon."
endif
	$(V1) $(CROSS_CC) -o $@ $^ $(LD_FLAGS)
	$(V1) $(SIZE) $(TARGET_ELF)

# Compile
ifeq ($(DEBUG),GDB)
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) echo "%% (debug) $<" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(CC_DEBUG_OPTIMISATION) $<
else
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) $(if $(findstring $(subst ./src/main/,,$<),$(SPEED_OPTIMISED_SRC)), \
	echo "%% (speed optimised) $<" "$(STDOUT)" && \
	$(CROSS_CC) -c -o $@ $(CFLAGS) $(CC_SPEED_OPTIMISATION) $<, \
	$(if $(findstring $(subst ./src/main/,,$<),$(SIZE_OPTIMISED_SRC)), \
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
targets-group-1: $(GROUP_1_TARGETS)

## targets-group-2   : build some targets
targets-group-2: $(GROUP_2_TARGETS)

## targets-group-3   : build some targets
targets-group-3: $(GROUP_3_TARGETS)

## targets-group-3   : build some targets
targets-group-4: $(GROUP_4_TARGETS)

## targets-group-rest: build the rest of the targets (not listed in group 1, 2 or 3)
targets-group-rest: $(GROUP_OTHER_TARGETS)

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
	rm -rf obj

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


flash_$(TARGET): $(TARGET_HEX)
	$(V0) stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	$(V0) echo -n 'R' >$(SERIAL_DEVICE)
	$(V0) stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## flash             : flash firmware (.hex) onto flight controller
flash: flash_$(TARGET)

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

unbrick_$(TARGET): $(TARGET_HEX)
	$(V0) stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	$(V0) stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

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

## targets           : print a list of all valid target platforms (for consumption by scripts)
targets:
	@echo "Valid targets:       $(VALID_TARGETS)"
	@echo "Variant targets:     $(VARIANT_TARGETS)"
	@echo "Supported targets:   $(SUPPORTED_TARGETS)"
	@echo "Unsupported targets: $(UNSUPPORTED_TARGETS)"
	@echo "Target:              $(TARGET)"
	@echo "Base target:         $(BASE_TARGET)"
	@echo "targets-group-1:     $(GROUP_1_TARGETS)"
	@echo "targets-group-2:     $(GROUP_2_TARGETS)"
	@echo "targets-group-3:     $(GROUP_3_TARGETS)"
	@echo "targets-group-4:     $(GROUP_4_TARGETS)"
	@echo "targets-group-rest:  $(GROUP_OTHER_TARGETS)"

## test              : run the cleanflight test suite
## junittest         : run the cleanflight test suite, producing Junit XML result files.
test junittest:
	$(V0) cd src/test && $(MAKE) $@


check-target-independence:
	$(V1) for test_target in $(VALID_TARGETS); do \
		FOUND=$$(grep -rE "\W$${test_target}\W?" src/main | grep -vE "(//)|(/\*).*\W$${test_target}\W?" | grep -vE "^src/main/target"); \
		if [ "$${FOUND}" != "" ]; then \
			echo "Target dependencies found:"; \
			echo "$${FOUND}"; \
			exit 1; \
		fi; \
	done

check-fastram-usage-correctness:
	$(V1) NON_TRIVIALLY_INITIALIZED=$$(grep -Ern "\W?FAST_RAM_ZERO_INIT\W.*=.*" src/main/ | grep -Ev "=\s*(false|NULL|0(\.0*f?)?)\s*[,;]"); \
	if [ "$${NON_TRIVIALLY_INITIALIZED}" != "" ]; then \
		echo "Non-trivially initialized FAST_RAM_ZERO_INIT variables found, use FAST_RAM instead:"; \
		echo "$${NON_TRIVIALLY_INITIALIZED}"; \
		exit 1; \
	fi; \
	TRIVIALLY_INITIALIZED=$$(grep -Ern "\W?FAST_RAM\W.*;" src/main/ | grep -v "="); \
	EXPLICITLY_TRIVIALLY_INITIALIZED=$$(grep -Ern "\W?FAST_RAM\W.*;" src/main/ | grep -E "=\s*(false|NULL|0(\.0*f?)?)\s*[,;]"); \
	if [ "$${TRIVIALLY_INITIALIZED}$${EXPLICITLY_TRIVIALLY_INITIALIZED}" != "" ]; then \
		echo "Trivially initialized FAST_RAM variables found, use FAST_RAM_ZERO_INIT instead to save FLASH:"; \
		echo "$${TRIVIALLY_INITIALIZED}\n$${EXPLICITLY_TRIVIALLY_INITIALIZED}"; \
		exit 1; \
	fi;

# rebuild everything when makefile changes
$(TARGET_OBJS) : Makefile

# include auto-generated dependencies
-include $(TARGET_DEPS)

commit:
	git commit -a -m="More Revo progress"
	git push
