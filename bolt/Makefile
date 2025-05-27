PYTHON            ?= python3

DFU_UTIL          ?= dfu-util

HACKFLIGHT = $(HOME)/Desktop/hackflight

HASKDIR = $(HACKFLIGHT)/haskell

ARCH_CFLAGS += -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3
ARCH_CFLAGS += -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee
ARCH_CFLAGS += -Wno-array-bounds -Wno-stringop-overread
ARCH_CFLAGS += -Wno-stringop-overflow
ARCH_CFLAGS += -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER

VENDOR = ./vendor

PROCESSOR = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

LINKER_DIR = $(srctree)/tools/make/F405/linker

LDFLAGS += --specs=nosys.specs --specs=nano.specs $(PROCESSOR) -nostdlib
image_LDFLAGS += -Wl,-Map=bolt.map,--cref,--gc-sections,--undefined=uxTopUsedPriority
image_LDFLAGS += -L$(srctree)/tools/make/F405/linker
image_LDFLAGS += -T $(LINKER_DIR)/FLASH_CLOAD.ld

CMSIS = $(VENDOR)/STMicroelectronics/tools/CMSIS/5.9.0/CMSIS

ARDUINO_LIBS = $(HOME)/Documents/Arduino/libraries

INCLUDES += -I$(ARDUINO_LIBS)

INCLUDES += -I$(ARDUINO_LIBS)/PMW3901/src/
INCLUDES += -I$(ARDUINO_LIBS)/VL53L1/src/
INCLUDES += -I$(ARDUINO_LIBS)/Arduino_CMSIS-DSP/src/

VL53L1 = $(ARDUINO_LIBS)/VL53L1/src

INCLUDES += -I$(VL53L1)/st

INCLUDES += -I$(srctree)/src/free_rtos/

INCLUDES += -I$(srctree)/src/stm32f4

INCLUDES += -I$(srctree)/src/hal

INCLUDES += -I$(CMSIS)/DSP/Include
INCLUDES += -I$(CMSIS)/Core/Include

INCLUDES += -I$(srctree)/src/deck/drivers/vl53l1

INCLUDES += -I$(srctree)/src/stm32f4/STM32F4xx_StdPeriph_Driver/inc
INCLUDES += -I$(srctree)/src/stm32f4/STM32_USB_Device_Library/Core/inc
INCLUDES += -I$(srctree)/src/stm32f4/STM32_USB_OTG_Driver/inc

INCLUDES += -I$(srctree)/src

INCLUDES += -I$(KBUILD_OUTPUT)/include/generated

objs-y += src
objs-y += $(VENDOR)
objs-y += $(VL53L1)

objs-y += $(OOT)

MEM_SIZE_FLASH_K = 1008
MEM_SIZE_RAM_K = 128
MEM_SIZE_CCM_K = 64

KBUILD_OUTPUT ?= build

-include $(KBUILD_OUTPUT)/include/config/auto.conf

PROG ?= bolt

ifeq ($(CONFIG_DEBUG),y)
ARCH_CFLAGS	+= -O0 -Wconversion
else
ARCH_CFLAGS += -Os -Werror
endif

_all:

all: $(PROG).hex $(PROG).bin
	@$(PYTHON) $(srctree)/tools/make/versionTemplate.py --crazyflie-base $(srctree) --print-version
	@$(PYTHON) $(srctree)/tools/make/size.py $(SIZE) $(PROG).elf $(MEM_SIZE_FLASH_K) $(MEM_SIZE_RAM_K) $(MEM_SIZE_CCM_K)

	for f in $$(ls $(PROG).*); do \
		ln -sf $(KBUILD_OUTPUT)/$$f $(srctree)/$$(basename $$f); \
	done

include tools/make/targets.mk

flash:
	$(DFU_UTIL) -d 0483:df11 -a 0 -s 0x08004000:leave -D $(PROG).bin

unbrick:
	$(DFU_UTIL) -d 0483:5740 -a 0 -s 0x08000000:leave -D $(PWD)/cf2loader-1.0.bin

# Give control over to Kbuild
-include tools/kbuild/Makefile.kbuild


standard_config:
	ln -sf $(HOME)/.arduino15/packages/STMicroelectronics/ vendor
	ln -sf $(ARDUINO_LIBS)/vl53l1x-arduino vendor
	cp $(HACKFLIGHT)/src/system.cpp $(HACKFLIGHT)/bolt/src/
	cp $(HACKFLIGHT)/src/control/standard.hpp $(HACKFLIGHT)/src/control.hpp
	cp $(HACKFLIGHT)/bolt/src/Kbuild.standard $(HACKFLIGHT)/bolt/src/Kbuild
	cd $(HACKFLIGHT)/msppg; make install; cd ..

haskell_config:
	ln -sf $(HOME)/.arduino15/packages/STMicroelectronics/ vendor
	ln -sf $(ARDUINO_LIBS)/vl53l1x-arduino vendor
	runhaskell -i$(HASKDIR) -i$(HASKDIR)/pids $(HASKDIR)/Pids.hs -i$(PWD)
	mv copilot_core.c $(HACKFLIGHT)/bolt/src/copilot_core.cpp
	mv copilot*.h $(HACKFLIGHT)/bolt/src/
	cp $(HACKFLIGHT)/src/system.cpp $(HACKFLIGHT)/bolt/src/
	cp $(HACKFLIGHT)/src/control/haskell.hpp $(HACKFLIGHT)/src/control.hpp
	cp $(HACKFLIGHT)/bolt/src/Kbuild.haskell $(HACKFLIGHT)/bolt/src/Kbuild
	cd $(HACKFLIGHT)/msppg; make install; cd ..

clean:
	rm -rf build/*.bin build/*.elf build/*.hex build/*.map build/include/ bolt.* \
		src/copilot* build/.firmware.cmd build/Makefile build/scripts build/vendor \
	 	src/Kbuild build/src build/source $(PROG).bin *.elf  *.hex *.map *.dfu copilot*
	#find . -name "*.o" -type f -delete
	#find . -name ".*.o*" -type f -delete
