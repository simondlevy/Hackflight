BREEZY_DIR = $(HOME)/Desktop/BreezySTM32

FIRMWARE_DIR = $(HOME)/Desktop/hackflight/firmware

PROJECT = Hackflight

# Change this to wherever you put Arduino libraries
ARDUINO_DIR = $(HOME)/Development/arduino-1.8.1/hardware/teensy/avr/libraries

# The target to build, see VALID_TARGETS below
TARGET		?= ALIENFLIGHTF3

# Compile-time options
OPTIONS		?=
export OPTIONS

# Debugger optons, must be empty or GDB
DEBUG ?=

# Serial port/Device for flashing
SERIAL_DEVICE	?= /dev/ttyACM0

# Flash size (KB).  Some low-end chips actually have more flash than advertised, use this to override.
FLASH_SIZE ?=

###############################################################################

F3_DIR = $(BREEZY_DIR)/f3
ALIENFLIGHTF3_DIR = $(F3_DIR)/alienflightf3
VCP_DIR = $(F3_DIR)/vcp

SPEKTRUM_DIR = $(ARDUINO_DIR)/SpektrumDSM

MPU6050_DIR = $(ARDUINO_DIR)/MPU6050

256K_TARGETS = ALIENFLIGHTF3 

F3_TARGETS = ALIENFLIGHTF3 

VALID_TARGETS = $(256K_TARGETS)

VCP_TARGETS = ALIENFLIGHTF3 

# Configure default flash sizes for the targets
ifeq ($(FLASH_SIZE),)
ifeq ($(TARGET),$(filter $(TARGET),$(64K_TARGETS)))
FLASH_SIZE = 64
else ifeq ($(TARGET),$(filter $(TARGET),$(128K_TARGETS)))
FLASH_SIZE = 128
else ifeq ($(TARGET),$(filter $(TARGET),$(256K_TARGETS)))
FLASH_SIZE = 256
else
$(error FLASH_SIZE not configured for target $(TARGET))
endif
endif

REVISION := $(shell git log -1 --format="%h")

# Working directories
#ROOT		 := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
ROOT		 = $(dir $(lastword $(MAKEFILE_LIST)))
SRC_DIR		 = $(ALIENFLIGHTF3_DIR)
OBJECT_DIR	 = $(ROOT)/obj
BIN_DIR		 = $(ROOT)/obj
#CMSIS_DIR	 = $(ROOT)/lib/CMSIS
CMSIS_DIR	 = $(BREEZY_DIR)/lib/CMSIS
INCLUDE_DIRS = $(BREEZY_DIR) $(F3_DIR) $(VCP_DIR) $(ALIENFLIGHTF3_DIR) $(SRC_DIR) $(SRC_DIR)/target $(FIRMWARE_DIR) .
LINKER_DIR	 = $(F3_DIR)

# Search path for sources
VPATH		:= $(SRC_DIR):$(SRC_DIR)/startup
USBFS_DIR	= $(BREEZY_DIR)/lib/STM32_USB-FS-Device_Driver
USBPERIPH_SRC = $(notdir $(wildcard $(USBFS_DIR)/src/*.c))

CSOURCES        := $(shell find $(SRC_DIR) -name '*.c')

# F3 TARGETS

STDPERIPH_DIR	= $(BREEZY_DIR)/lib/STM32F30x_StdPeriph_Driver

STDPERIPH_SRC = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

EXCLUDES	= stm32f30x_crc.c stm32f30x_can.c

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

DEVICE_STDPERIPH_SRC = $(STDPERIPH_SRC)


VPATH		:= $(VPATH):$(CMSIS_DIR)/CM1/CoreSupport:$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM1/CoreSupport/*.c \
			   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(STDPERIPH_DIR)/inc \
		   $(SPEKTRUM_DIR) \
		   $(MPU6050_DIR) \
		   $(CMSIS_DIR)/CM1/CoreSupport \
		   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x

ifeq ($(TARGET),$(filter $(TARGET),$(VCP_TARGETS)))
INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(USBFS_DIR)/inc \
		   $(SRC_DIR)/vcp

VPATH := $(VPATH):$(USBFS_DIR)/src

DEVICE_STDPERIPH_SRC := $(DEVICE_STDPERIPH_SRC)\
		   $(USBPERIPH_SRC)

endif

LD_SCRIPT	 = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS = -DSTM32F303xC -DSTM32F303
TARGET_FLAGS = -D$(TARGET)

ifneq ($(FLASH_SIZE),)
DEVICE_FLAGS := $(DEVICE_FLAGS) -DFLASH_SIZE=$(FLASH_SIZE)
endif

TARGET_DIR = $(ROOT)/target/$(TARGET)
TARGET_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))

# VARIANTS
ifeq ($(TARGET),ALIENFLIGHTF1)
# ALIENFLIGHTF1 is a VARIANT of NAZE
TARGET_FLAGS := $(TARGET_FLAGS) -DNAZE -DALIENFLIGHT
TARGET_DIR = $(ROOT)/target/NAZE
endif
ifeq ($(TARGET),CHEBUZZF3)
# CHEBUZZ is a VARIANT of STM32F3DISCOVERY
TARGET_FLAGS := $(TARGET_FLAGS) -DSTM32F3DISCOVERY
endif
ifeq ($(TARGET),$(filter $(TARGET),RMDO IRCFUSIONF3))
# RMDO and IRCFUSIONF3 are a VARIANT of SPRACINGF3
TARGET_FLAGS := $(TARGET_FLAGS) -DSPRACINGF3
endif

# SMARTPORT F3
# Use bidirectional feature of F3 processor for smartport, for F1 targets always use two pins to allow external inverter.
ifeq ($(TARGET),$(filter $(TARGET),$(F3_TARGETS)))
TARGET_FLAGS := $(TARGET_FLAGS) -DUSE_SMARTPORT_ONEWIRE
endif

INCLUDE_DIRS := \
		    $(INCLUDE_DIRS) \
		    $(TARGET_DIR)

VPATH		:= $(VPATH):$(TARGET_DIR)

SYSTEM_SRC = \
		   $(F3_DIR)/dma.c \
		   $(F3_DIR)/serial.c \
		   $(F3_DIR)/system.c \
		   $(TARGET_SRC) \
		   $(CMSIS_SRC) \
		   $(DEVICE_STDPERIPH_SRC)

FC_COMMON_SRC = \
   	       $(F3_DIR)/exti.c \
		   $(F3_DIR)/io.c \

VCP_SRC = \
		   $(VCP_DIR)/hw_config.c \
		   $(VCP_DIR)/stm32_it.c \
		   $(VCP_DIR)/usb_desc.c \
		   $(VCP_DIR)/usb_endp.c \
		   $(VCP_DIR)/usb_istr.c \
		   $(VCP_DIR)/usb_prop.c \
		   $(VCP_DIR)/usb_pwr.c \
		   $(F3_DIR)/serial_usb_vcp.c \
		   $(F3_DIR)/usb_io.c

STM32F30x_COMMON_SRC = \
		   $(F3_DIR)/startup_stm32f30x_md_gcc.S \
		   $(F3_DIR)/system_stm32f30x_extras.c \
		   $(F3_DIR)/gpio_stm32f30x.c \
		   $(F3_DIR)/serial_uart.c \
		   $(F3_DIR)/serial_uart_stm32f30x.c \
		   $(F3_DIR)/system_stm32f30x.c

ALIENFLIGHTF3_SRC = \
		   $(SYSTEM_SRC) \
		   $(FC_COMMON_SRC) \
		   $(STM32F30x_COMMON_SRC) \
		   $(F3_DIR)/timer_stm32f30x.c \
		   $(ALIENFLIGHTF3_DIR)/timer.c \
		   $(ALIENFLIGHTF3_DIR)/target.c \
		   $(VCP_SRC)

VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src

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
C++         := $(CCACHE) arm-none-eabi-g++
CC          := $(CCACHE) arm-none-eabi-gcc
OBJCOPY     := arm-none-eabi-objcopy
SIZE        := arm-none-eabi-size

#
# Tool options.
#

ifeq ($(DEBUG),GDB)
OPTIMIZE	 = -O0
LTO_FLAGS	 = $(OPTIMIZE)
else
OPTIMIZE	 = -Os
LTO_FLAGS	 =  -flto -fuse-linker-plugin $(OPTIMIZE)
endif

ifneq ($(filter $(OPTIONS),FAIL_ON_WARNINGS),)
WARN_FLAGS      += -Werror
endif

DEBUG_FLAGS	 = -ggdb3 -DDEBUG

CFLAGS		 = $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(WARN_FLAGS) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   $(DEBUG_FLAGS) \
		   $(DEVICE_FLAGS) \
		   -Wall -Wpedantic -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion -Wundef \
		   -ffunction-sections \
		   -fdata-sections \
		   -DUSE_STDPERIPH_DRIVER

ASFLAGS		 = $(ARCH_FLAGS) \
		   $(WARN_FLAGS) \
		   -x assembler-with-cpp \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		  -MMD -MP

#LDFLAGS		 = -lm \
		   -nostartfiles \
		   -lc \
		   --specs=rdimon.specs \
		   $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(DEBUG_FLAGS) \
		   -static \
		   -Wl,-gc-sections,-Map,$(TARGET_MAP) \
		   -Wl,-L$(LINKER_DIR) \
		   -T$(LD_SCRIPT)

LDFLAGS = -lm \
		   -nostartfiles \
		   -lc \
		   --specs=nano.specs \
		   -lnosys \
		   $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(DEBUG_FLAGS) \
		   -static \
		   -Wl,-gc-sections,-Map,$(TARGET_MAP) \
		   -Wl,-L$(LINKER_DIR) \
		   -T$(LD_SCRIPT)

###############################################################################
# No user-serviceable parts below
###############################################################################

CPPCHECK         = cppcheck $(CSOURCES) --enable=all --platform=unix64 \
		   --std=c99 --inline-suppr --quiet --force \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   -I/usr/include -I/usr/include/linux

#
# Things we will build
#
ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS))
endif

TARGET_BIN	 = obj/$(TARGET).bin
TARGET_HEX	 = $(BIN_DIR)/$(TARGET).hex
TARGET_ELF	 = $(OBJECT_DIR)/$(TARGET).elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC)))) \
			   $(OBJECT_DIR)/$(TARGET)/hackflight.o \
			   $(OBJECT_DIR)/$(TARGET)/alienflightf3.o \
			   $(OBJECT_DIR)/$(TARGET)/Arduino.o \
			   $(OBJECT_DIR)/$(TARGET)/HardwareWire.o \
			   $(OBJECT_DIR)/$(TARGET)/SpektrumDSM.o \
			   $(OBJECT_DIR)/$(TARGET)/MPU6050.o \
			   $(OBJECT_DIR)/$(TARGET)/Motor.o
TARGET_DEPS	 = $(addsuffix .d,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_MAP	 = $(OBJECT_DIR)/$(TARGET).map


## Default make goal:
## hex         : Make filetype hex only
.DEFAULT_GOAL := bin

## Optional make goals:
## all         : Make all filetypes, binary and hex
all: hex bin

## binary      : Make binary filetype
## bin         : Alias of 'binary'
## hex         : Make hex filetype
bin:    $(TARGET_BIN)
binary: $(TARGET_BIN)
hex:    $(TARGET_HEX)

# rule to reinvoke make with TARGET= parameter
# rules that should be handled in toplevel Makefile, not dependent on TARGET
GLOBAL_GOALS	= all_targets cppcheck test

.PHONY: $(VALID_TARGETS)
$(VALID_TARGETS):
	$(MAKE) TARGET=$@ $(filter-out $(VALID_TARGETS) $(GLOBAL_GOALS), $(MAKECMDGOALS))

## all_targets : Make all TARGETs
.PHONY: all_targets
all_targets : $(VALID_TARGETS)

## clean       : clean up all temporary / machine-generated files
clean:
	rm -rf obj

flash_$(TARGET): $(TARGET_BIN)
	echo -n 'R' >$(SERIAL_DEVICE)
	sleep 2
	dfu-util -a 0 -s 0x08000000:leave -D $(TARGET_BIN)

## flash       : flash firmware (.hex) onto flight controller
flash: flash_$(TARGET)

unbrick_$(TARGET): $(TARGET_BIN)
	dfu-util -a 0 -s 0x08000000:leave -D $(TARGET_BIN)

## unbrick     : unbrick flight controller
unbrick: unbrick_$(TARGET)

## cppcheck    : run static analysis on C source code
cppcheck: $(CSOURCES)
	$(CPPCHECK)

cppcheck-result.xml: $(CSOURCES)
	$(CPPCHECK) --xml-version=2 2> cppcheck-result.xml


# rebuild everything when makefile changes
$(TARGET_OBJS) : Makefile

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_BIN): $(TARGET_ELF)
	$(OBJCOPY) -O binary $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(C++) -o $@ $^ $(LDFLAGS)

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<


$(OBJECT_DIR)/$(TARGET)/hackflight.o: hackflight.cpp $(FIRMWARE_DIR)/*.hpp
	@$(CC) -c -std=gnu++11 -o $(OBJECT_DIR)/$(TARGET)/hackflight.o $(CFLAGS) hackflight.cpp 

$(OBJECT_DIR)/$(TARGET)/alienflightf3.o: alienflightf3.cpp $(FIRMWARE_DIR)/*.hpp
	@$(CC) -c -std=gnu++11 -o $(OBJECT_DIR)/$(TARGET)/alienflightf3.o $(CFLAGS) alienflightf3.cpp 

$(OBJECT_DIR)/$(TARGET)/Arduino.o: $(F3_DIR)/Arduino.cpp $(BREEZY_DIR)/Arduino.h
	@$(CC) -c -o $(OBJECT_DIR)/$(TARGET)/Arduino.o $(CFLAGS) $(F3_DIR)/Arduino.cpp

$(OBJECT_DIR)/$(TARGET)/HardwareWire.o: $(F3_DIR)/HardwareWire.cpp $(BREEZY_DIR)/Wire.h
	@$(CC) -c -o $(OBJECT_DIR)/$(TARGET)/HardwareWire.o $(CFLAGS) $(F3_DIR)/HardwareWire.cpp

$(OBJECT_DIR)/$(TARGET)/Motor.o: $(F3_DIR)/Motor.cpp 
	@$(CC) -c -o $(OBJECT_DIR)/$(TARGET)/Motor.o $(CFLAGS) $(F3_DIR)/Motor.cpp

$(OBJECT_DIR)/$(TARGET)/SpektrumDSM.o: $(SPEKTRUM_DIR)/SpektrumDSM.cpp $(SPEKTRUM_DIR)/SpektrumDSM.h
	@$(CC) -c -o $(OBJECT_DIR)/$(TARGET)/SpektrumDSM.o $(CFLAGS) $(SPEKTRUM_DIR)/SpektrumDSM.cpp

$(OBJECT_DIR)/$(TARGET)/MPU6050.o: $(MPU6050_DIR)/MPU6050.cpp $(MPU6050_DIR)/MPU6050.h
	@$(CC) -c -o $(OBJECT_DIR)/$(TARGET)/MPU6050.o $(CFLAGS) $(MPU6050_DIR)/MPU6050.cpp

commit:
	git commit -a --allow-empty-message -m ''
	git push

listen:
	miniterm.py $(SERIAL_DEVICE)
