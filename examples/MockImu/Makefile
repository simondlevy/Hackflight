TARGET_MCU := STM32F4

TARGET    := STM32F405

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

ARM_SDK_DIR ?= $(TOOLS_DIR)/gcc-arm-11.2-2022.02-x86_64-arm-none-eabi
GCC_REQUIRED_VERSION ?= 9.3.1
STM32FLASH_DIR := $(TOOLS_DIR)/stm32flash
ARM_SDK_PREFIX := $(ARM_SDK_DIR)/bin/arm-none-eabi-

# default xtal value for F4 targets
HSE_VALUE       ?= 8000000

# used to disable features based on flash space shortage (larger number => more
# features disabled)
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
STM32_SRC = \
            serial_debug.c \
            bitbang.c \
            spi.c \
            spi_stm32f405.c \
            dma_common.c \
            dma_reqmap_f405.c \
            exti.c \
            io.c \
            pinio.c \
            rcc.c \
            serial.c \
            serial_uart.c \
            serial_uart_pinconfig.c \
            serial_uart_stm32f405.c \
            system.c \
            timer_stm32f405.c \
            $(addprefix pg/, $(notdir $(wildcard $(STM_SRC_DIR)/pg/*.c))) \
            $(addprefix common/,$(notdir $(wildcard $(STM_SRC_DIR)/common/*.c))) \
            $(addprefix config/,$(notdir $(wildcard $(STM_SRC_DIR)/config/*.c)))

COMMON_DEVICE_SRC =  $(CMSIS_SRC) $(DEVICE_STDPERIPH_SRC)
COMMON_SRC := $(STM32_SRC) $(COMMON_DEVICE_SRC)


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
              --specs=nosys.specs \
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
	rm -rf src
	rm -rf ../src

$(TARGET_DFU): $(TARGET_HEX)
	@echo "Creating DFU $(TARGET_DFU)" "$(STDOUT)"
	$(V1) $(PYTHON) $(DFUSE-PACK) -i $< $@

$(TARGET_ELF): $(TARGET_OBJS) \
		  $(OBJECT_DIR)/hardware_init.o \
		  $(OBJECT_DIR)/main.o \
		  $(OBJECT_DIR)/stm32clock.o \
		  $(OBJECT_DIR)/dshot_packet.o
	@echo "Linking" $@
	$(V1) $(CROSS_CXX) -o $@ $(filter-out %.ld,$^) $(LD_FLAGS)

# C ===========================================================================

$(OBJECT_DIR)/hardware_init.o: hardware_init.c \
	  hardware_init.h \
      spi.h \
      exti.h \
      inverter.h \
      io.h \
      pinio.h \
      serialdev.h \
      serial_uart.h \
      systemdev.h \
      timer.h \
      usb_io.h
	@echo gcc hardware_init.c
	$(V1) $(CROSS_CC) -c -o $(OBJECT_DIR)/hardware_init.o $(CFLAGS) \
		hardware_init.c

$(OBJECT_DIR)/mpu6000.o: $(HFSRC_DIR)/imus/softquat/*.*
	@echo imus/softquat/mpu6000.cpp
	$(V1) $(call compile_cpp)

$(OBJECT_DIR)/dshot_packet.o: $(HFSRC_DIR)/escs/dshot/dshot_packet.c
	@echo escs/dshot/dshot_packet.c
	$(V1) $(CROSS_CC) -c -o $(OBJECT_DIR)/dshot_packet.o $(CFLAGS) \
		$(HFSRC_DIR)/escs/dshot/dshot_packet.c

$(OBJECT_DIR)/stm32clock.o: $(HFSRC_DIR)/stm32clock.c
	@echo stm32clock.c
	$(V1) $(CROSS_CC) -c -o $(OBJECT_DIR)/stm32clock.o $(CFLAGS) \
		$(HFSRC_DIR)/stm32clock.c

$(OBJECT_DIR)/main.o: main.cpp \
	  hardware_init.h \
	  $(STM_SRC_DIR)/*.h \
	  $(HFSRC_DIR)/*.h \
      $(HFSRC_DIR)/*/*.h \
      $(HFSRC_DIR)/*/*/*.h \
      $(HFSRC_DIR)/*/*/*/*.h
	$(V1) $(call compile_cpp)

define compile_c
	echo gcc "$<" "$(STDOUT)" && $(CROSS_CC) -c -o $@ $(CFLAGS) \
		$(CC_DEFAULT_OPTIMISATION) $<
endef

define compile_cpp
	echo g++ "$<" "$(STDOUT)" && $(CROSS_CXX) -c -std=c++20 -o $@ \
		$(CFLAGS) -Wno-register -Wno-volatile $(CC_DEFAULT_OPTIMISATION) $<
endef

$(TARG_OBJ_DIR)/%.o: %.c
	$(V1) mkdir -p $(dir $@)
	$(V1) $(call compile_c)

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
	miniterm $(SERIAL_DEVICE) 115200

edit:
	vim  main.cpp
