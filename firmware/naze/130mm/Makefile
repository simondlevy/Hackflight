###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################

# Change this to wherever you put BreezySTM32
BREEZY_DIR = $(HOME)/Desktop/BreezySTM32

###############################################################################
# Things that the user might override on the commandline
#

TARGET		?= NAZE

CPP_OBJS = mw.o imu.o mixer.o pid.o msp.o rc.o board.o

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
NAZE_SRC = $(BREEZY_DIR)/main.c \
           $(BREEZY_DIR)/printf.c \
           $(BREEZY_DIR)/drv_system.c \
           $(BREEZY_DIR)/drv_mpu6050.c \
           $(BREEZY_DIR)/drv_ms5611.c \
           $(BREEZY_DIR)/drv_mb1242.c \
           $(BREEZY_DIR)/drv_i2c.c \
           $(BREEZY_DIR)/drv_serial.c \
           $(BREEZY_DIR)/drv_pwm.c \
           $(BREEZY_DIR)/drv_gpio.c \
           $(BREEZY_DIR)/drv_uart.c \
           $(BREEZY_DIR)/drv_timer.c \
           $(BREEZY_DIR)/startup_stm32f10x_md_gcc.S \
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
			   $(BREEZY_DIR) \
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

CFLAGS		 = $(ARCH_FLAGS) \
		   $(LTO_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   $(DEBUG_FLAGS) \
		   -Wall -pedantic -Wextra -Wshadow -Wunsafe-loop-optimizations \
		   -ffunction-sections \
		   -fdata-sections \
		   -DSTM32F10X_MD \
		   -DUSE_STDPERIPH_DRIVER \
		   -D$(TARGET)

ASFLAGS		 = $(ARCH_FLAGS) \
		   -x assembler-with-cpp \
		   $(addprefix -I,$(INCLUDE_DIRS))

# XXX Map/crossref output?
LD_SCRIPT	 = $(BREEZY_DIR)/stm32_flash.ld
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

mw.o: ../../mw.cpp ../../mw.hpp
	@echo %% $(notdir $<)
	@$(CC) $(CFLAGS) -c -o mw.o ../../mw.cpp

imu.o: ../../imu.cpp ../../imu.hpp
	@echo %% $(notdir $<)
	@$(CC) $(CFLAGS) -c -o imu.o ../../imu.cpp

mixer.o: ../../mixer.cpp ../../mixer.hpp
	@echo %% $(notdir $<)
	@$(CC) $(CFLAGS) -c -o mixer.o ../../mixer.cpp

pid.o: ../../pid.cpp ../../pid.hpp
	@echo %% $(notdir $<)
	@$(CC) $(CFLAGS) -c -o pid.o ../../pid.cpp

msp.o: ../../msp.cpp ../../msp.hpp
	@echo %% $(notdir $<)
	@$(CC) $(CFLAGS) -c -o msp.o ../../msp.cpp

rc.o: ../../rc.cpp ../../rc.hpp
	@echo %% $(notdir $<)
	@$(CC) $(CFLAGS) -c -o rc.o ../../rc.cpp

board.o: ../board.cpp ../../board.hpp
	@echo %% $(notdir $<)
	@$(CC) $(CFLAGS) -c -o board.o ../board.cpp

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
