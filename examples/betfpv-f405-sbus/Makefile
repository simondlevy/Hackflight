TARGET_MCU := STM32F4

TARGET    := STM32F405

# SERIAL_DEVICE ?= /dev/ttyACM0
SERIAL_DEVICE ?= /dev/ttyACM0

# Working directories
ROOT         := ../../stm32f4
OBJECT_DIR   := ./obj
STM_SRC_DIR  := $(ROOT)/src
HFSRC_DIR    := ../../src
CMSIS_DIR    := $(ROOT)/lib/CMSIS
INCLUDE_DIRS := $(STM_SRC_DIR) \
	            $(HFSRC_DIR) \
                $(STM_SRC_DIR)/startup
LINKER_DIR   := $(ROOT)/link

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

########################################################

# developer preferences, edit these at will, they'll be gitignored
-include $(ROOT)/make/local.mk

# configure some directories that are relative to wherever ROOT_DIR is located
TOOLS_DIR ?= /opt

#ARM_SDK_DIR ?= $(TOOLS_DIR)/gcc-arm-none-eabi-9-2020-q2-update
ARM_SDK_DIR ?= $(TOOLS_DIR)/gcc-arm-11.2-2022.02-x86_64-arm-none-eabi
GCC_REQUIRED_VERSION ?= 9.3.1
STM32FLASH_DIR := $(TOOLS_DIR)/stm32flash
ARM_SDK_PREFIX := $(ARM_SDK_DIR)/bin/arm-none-eabi-

# default xtal value for F4 targets
HSE_VALUE       ?= 8000000

# used to disable features based on flash space shortage (larger number => more features disabled)
FEATURE_CUT_LEVEL_SUPPLIED := $(FEATURE_CUT_LEVEL)
FEATURE_CUT_LEVEL =

ALT_TARGET_PATHS  = $(filter-out %/target,$(basename $(wildcard $(STM_SRC_DIR)/target/*/*.mk)))
ALT_TARGET_NAMES  = $(notdir $(ALT_TARGET_PATHS))
BASE_TARGET_NAMES = $(notdir $(patsubst %/,%,$(dir $(ALT_TARGET_PATHS))))
BASE_ALT_PAIRS    = $(join $(BASE_TARGET_NAMES:=/),$(ALT_TARGET_NAMES))

find_target_pair  = $(filter %/$(1),$(BASE_ALT_PAIRS))
get_base_target   = $(if $(call find_target_pair,$(1)),$(patsubst %/,%,$(dir $(call find_target_pair,$(1)))),$(1))

UNIFIED_TARGETS := STM32F405

BASE_TARGET   := $(call get_base_target,$(TARGET))

F4_TARGETS      := $(F405_TARGETS) $(F411_TARGETS) $(F446_TARGETS)

TARGET_MCU := STM32F4

TARGET_FLAGS  	:= $(TARGET_FLAGS) -D$(TARGET_MCU)
REVISION := norevision
ifeq ($(shell git diff --shortstat),)
REVISION := $(shell git log -1 --format="%h")
endif

# Search path for sources
VPATH           := $(STM_SRC_DIR):$(STM_SRC_DIR)/startup
USBFS_DIR       = $(ROOT)/lib/STM32_USB-FS-Device_Driver
USBPERIPH_SRC   = $(notdir $(wildcard $(USBFS_DIR)/src/*.c))
FATFS_DIR       = $(ROOT)/lib/FatFS
FATFS_SRC       = $(notdir $(wildcard $(FATFS_DIR)/*.c))

CSOURCES        := $(shell find $(STM_SRC_DIR) -name '*.c')

# start specific includes
include $(ROOT)/STM32F4.mk

# Configure default flash sizes for the targets (largest size specified gets hit first) if flash not specified already.
ifeq ($(TARGET_FLASH_SIZE),)
ifneq ($(MCU_FLASH_SIZE),)
TARGET_FLASH_SIZE := $(MCU_FLASH_SIZE)
else
$(error MCU_FLASH_SIZE not configured for target $(TARGET))
endif
endif

DEVICE_FLAGS  := $(DEVICE_FLAGS) -DTARGET_FLASH_SIZE=$(TARGET_FLASH_SIZE)

ifneq ($(HSE_VALUE),)
DEVICE_FLAGS  := $(DEVICE_FLAGS) -DHSE_VALUE=$(HSE_VALUE)
endif

ifneq ($(FEATURE_CUT_LEVEL_SUPPLIED),)
DEVICE_FLAGS  := $(DEVICE_FLAGS) -DFEATURE_CUT_LEVEL=$(FEATURE_CUT_LEVEL_SUPPLIED)
else ifneq ($(FEATURE_CUT_LEVEL),)
DEVICE_FLAGS  := $(DEVICE_FLAGS) -DFEATURE_CUT_LEVEL=$(FEATURE_CUT_LEVEL)
endif

.DEFAULT_GOAL := hex

#include $(ROOT)/make/csource.mk
COMMON_SRC = \
            ./main.c \
            $(HFSRC_DIR)/imu.c \
            $(HFSRC_DIR)/led.c \
            $(HFSRC_DIR)/maths.c \
            $(HFSRC_DIR)/msp.c \
            $(HFSRC_DIR)/rx.c \
            $(HFSRC_DIR)/rx/dsmx.c \
            $(HFSRC_DIR)/rx/sbus.c \
            $(HFSRC_DIR)/stm32_clock.c \
            imu_mpu6000.c \
            dshot.c \
            dshot_dpwm.c \
            dshot_command.c \
            bus.c \
            bus_spi.c \
            bus_spi_config.c \
            bus_spi_pinconfig.c \
            dma_common.c \
            dma_reqmap.c \
            exti.c \
            flash.c \
            io.c \
            leddev.c \
            motor.c \
            pinio.c \
            rcc.c \
            serial.c \
            serial_uart.c \
            serial_uart_pinconfig.c \
            system.c \
            timer_common.c \
            timer.c \
            pwm_output.c \
            $(addprefix pg/, $(notdir $(wildcard $(STM_SRC_DIR)/pg/*.c))) \
            $(addprefix common/,$(notdir $(wildcard $(STM_SRC_DIR)/common/*.c))) \
            $(addprefix config/,$(notdir $(wildcard $(STM_SRC_DIR)/config/*.c)))

COMMON_DEVICE_SRC =  $(CMSIS_SRC) $(DEVICE_STDPERIPH_SRC)
COMMON_SRC := $(COMMON_SRC) $(COMMON_DEVICE_SRC)


# Search path and source files for the ST stdperiph library
VPATH        := $(VPATH):$(STDPERIPH_DIR)/src

SRC := $(STARTUP_SRC) $(MCU_COMMON_SRC) $(TARGET_SRC) 

SRC += $(COMMON_SRC)

#excludes
SRC   := $(filter-out $(MCU_EXCLUDES), $(SRC)) $(VCP_SRC)

######################################################33

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
DFUSE-PACK  := $(ROOT)/utils/dfuse-pack.py

#
# Tool options.
#
CC_DEFAULT_OPTIMISATION := $(OPTIMISATION_BASE) $(OPTIMISE_DEFAULT)
CC_SPEED_OPTIMISATION   := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
CC_SIZE_OPTIMISATION    := $(OPTIMISATION_BASE) $(OPTIMISE_SIZE)
CC_NO_OPTIMISATION      := 

CFLAGS     += $(ARCH_FLAGS) \
              $(addprefix -I,$(INCLUDE_DIRS)) \
              -std=gnu11 \
              -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion \
			  -Wno-unused-function \
              -ffunction-sections \
              -fdata-sections \
              -fno-common \
              -pedantic \
              $(TEMPORARY_FLAGS) \
              $(DEVICE_FLAGS) \
              -D_GNU_SOURCE \
              -DUSE_STDPERIPH_DRIVER \
              -D$(TARGET) \
              $(TARGET_FLAGS) \
              -save-temps=obj \
              -MMD -MP \
              $(EXTRA_FLAGS)

ASFLAGS     = $(ARCH_FLAGS) \
              -x assembler-with-cpp \
              $(addprefix -I,$(INCLUDE_DIRS)) \
              -MMD -MP

LD_FLAGS     = -lm \
              -nostartfiles \
              --specs=nano.specs \
              -lc \
              -lnosys \
              $(ARCH_FLAGS) \
              $(LTO_FLAGS) \
              -static \
              -Wl,-gc-sections,-Map,$(TARGET_MAP) \
              -Wl,-L$(LINKER_DIR) \
              -Wl,--cref \
              -Wl,--no-wchar-size-warning \
              -Wl,--print-memory-usage \
              -T$(LD_SCRIPT)


TARGET_BASENAME = $(OBJECT_DIR)/hackflight

TARG_OBJ_DIR = $(OBJECT_DIR)/$(TARGET)

#
# Things we will build
#
TARGET_BIN      = $(TARGET_BASENAME).bin
TARGET_HEX      = $(TARGET_BASENAME).hex
TARGET_DFU      = $(TARGET_BASENAME).dfu
TARGET_ELF      = $(TARG_OBJ_DIR).elf
TARGET_OBJS     = $(addsuffix .o,$(addprefix $(TARG_OBJ_DIR)/,$(basename $(SRC))))
TARGET_DEPS     = $(addsuffix .d,$(addprefix $(TARG_OBJ_DIR)/,$(basename $(SRC))))
TARGET_MAP      = $(TARG_OBJ_DIR).map

CLEAN_ARTIFACTS := $(TARGET_BIN)
CLEAN_ARTIFACTS += $(TARGET_HEX)
CLEAN_ARTIFACTS += $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)
CLEAN_ARTIFACTS += $(TARGET_DFU)

# Make sure build date and revision is updated on every incremental build
$(TARG_OBJ_DIR)/build/version.o : $(SRC)

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_LST): $(TARGET_ELF)
	$(V0) $(OBJDUMP) -S --disassemble $< > $@

$(TARGET_BIN): $(TARGET_ELF)
	@echo "Creating BIN $(TARGET_BIN)" "$(STDOUT)"
	$(V1) $(OBJCOPY) -O binary $< $@
	
$(TARGET_HEX): $(TARGET_ELF)
	@echo "Creating HEX $(TARGET_HEX)" "$(STDOUT)"
	$(V1) $(OBJCOPY) -O ihex --set-start 0x8000000 $< $@
	rm -rf ../src

$(TARGET_DFU): $(TARGET_HEX)
	@echo "Creating DFU $(TARGET_DFU)" "$(STDOUT)"
	$(V1) $(PYTHON) $(DFUSE-PACK) -i $< $@

$(TARGET_ELF): $(TARGET_OBJS) \
		  $(OBJECT_DIR)/arming.o \
		  $(OBJECT_DIR)/debug.o \
		  $(OBJECT_DIR)/failsafe.o \
		  $(OBJECT_DIR)/gyro.o \
		  $(OBJECT_DIR)/hackflight.o
	@echo "Linking" $@
	$(V1) $(CROSS_CC) -o $@ $(filter-out %.ld,$^) $(LD_FLAGS)

$(OBJECT_DIR)/arming.o: $(HFSRC_DIR)/arming.c \
      $(HFSRC_DIR)/arming.h \
      $(HFSRC_DIR)/datatypes.h \
      $(HFSRC_DIR)/failsafe.h \
      $(HFSRC_DIR)/led.h \
      $(HFSRC_DIR)/motor.h \
      $(HFSRC_DIR)/rx_status.h
	@echo arming.c
	$(V1) $(CROSS_CC) -c -o $(OBJECT_DIR)/arming.o $(CFLAGS) \
		$(HFSRC_DIR)/arming.c

$(OBJECT_DIR)/debug.o: $(HFSRC_DIR)/debug.c \
	  $(HFSRC_DIR)/msp.h \
	  $(HFSRC_DIR)/datatypes.h
	@echo debug.c
	$(V1) $(CROSS_CC) -c -o $(OBJECT_DIR)/debug.o $(CFLAGS) $(HFSRC_DIR)/debug.c

$(OBJECT_DIR)/failsafe.o: $(HFSRC_DIR)/failsafe.c \
	  $(HFSRC_DIR)/arming.h \
	  $(HFSRC_DIR)/debug.h \
	  $(HFSRC_DIR)/failsafe.h \
	  $(HFSRC_DIR)/rx.h \
	  $(HFSRC_DIR)/rx_status.h \
	  $(HFSRC_DIR)/time.h
	@echo failsafe.c
	$(V1) $(CROSS_CC) -c -o $(OBJECT_DIR)/failsafe.o $(CFLAGS) $(HFSRC_DIR)/failsafe.c

$(OBJECT_DIR)/gyro.o: $(HFSRC_DIR)/gyro.c \
	  $(HFSRC_DIR)/core_rate.h \
	  $(HFSRC_DIR)/datatypes.h \
	  $(HFSRC_DIR)/debug.h \
	  $(HFSRC_DIR)/gyro.h \
	  $(HFSRC_DIR)/imu.h \
	  $(HFSRC_DIR)/maths.h \
	  $(HFSRC_DIR)/pt1_filter.h \
	  $(HFSRC_DIR)/stats.h \
	  $(HFSRC_DIR)/system.h \
	  $(HFSRC_DIR)/time.h
	@echo gyro.c
	$(V1) $(CROSS_CC) -c -o $(OBJECT_DIR)/gyro.o $(CFLAGS) $(HFSRC_DIR)/gyro.c

$(OBJECT_DIR)/hackflight.o: $(HFSRC_DIR)/hackflight.c \
	  $(HFSRC_DIR)/arming.h \
	  $(HFSRC_DIR)/debug.h \
	  $(HFSRC_DIR)/gyro.h \
	  $(HFSRC_DIR)/hackflight.h \
	  $(HFSRC_DIR)/led.h \
	  $(HFSRC_DIR)/msp.h \
	  $(HFSRC_DIR)/rx.h \
	  $(HFSRC_DIR)/system.h
	@echo hackflight.c
	$(V1) $(CROSS_CC) -c -o $(OBJECT_DIR)/hackflight.o $(CFLAGS) $(HFSRC_DIR)/hackflight.c

define compile_file
	echo "$<" "$(STDOUT)" && $(CROSS_CC) -c -o $@ $(CFLAGS) $(CC_DEFAULT_OPTIMISATION) $<
endef

$(TARG_OBJ_DIR)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) $(call compile_file)

# Assemble
$(TARG_OBJ_DIR)/%.o: %.s
	$(V1) mkdir -p $(dir $@)
	@echo "%% $(notdir $<)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<

$(TARG_OBJ_DIR)/%.o: %.S
	$(V1) mkdir -p $(dir $@)
	@echo "%% $(notdir $<)" "$(STDOUT)"
	$(V1) $(CROSS_CC) -c -o $@ $(ASFLAGS) $<

$(VALID_TARGETS):
	$(V0) @echo "Building $@" && \
	$(MAKE) binary hex TARGET=$@ && \
	echo "Building $@ succeeded."

clean:
	rm -rf obj

TARGETS_FLASH = $(addsuffix _flash,$(VALID_TARGETS))

flash: $(TARGET_DFU)
	$(V0) echo -n 'R' > $(SERIAL_DEVICE)
	$(V0) sleep 1
	$(V0) $(MAKE) $(TARGET_DFU)
	$(V0) dfu-util -a 0 -D $(TARGET_DFU) -s :leave

unbrick: $(TARGET_DFU)
	$(V0) dfu-util -a 0 -D $(TARGET_DFU) -s :leave

binary:
	$(V0) $(MAKE) -j $(TARGET_BIN)

hex:
	$(V0) $(MAKE) -j $(TARGET_HEX)

$(TARGET_OBJS): $(wildcard make/*)

# include auto-generated dependencies
-include $(TARGET_DEPS)

listen:
	miniterm.py $(SERIAL_DEVICE) 115200
