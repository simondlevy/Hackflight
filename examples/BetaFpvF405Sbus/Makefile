#   Copyright (c) 2023 Simon D. Levy
#
#   This file is part of Hackflight.
#
#   Hackflight is free software: you can redistribute it and/or modify it under
#   the terms of the GNU General Public License as published by the Free
#   Software Foundation, either version 3 of the License, or (at your option)
#   any later version.
#
#   Hackflight is distributed in the hope that it will be useful, but WITHOUT
#   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
#   more details.
#
#   You should have received a copy of the GNU General Public License along with
#   Hackflight. If not, see <https://www.gnu.org/licenses/>.

SKETCH = BetaFpvF405Sbus

RUSTLIB = ./target/thumbv7em-none-eabihf/debug/libhackflight.a

GEN = F4
PNUM = GENERIC_F405RGTX
VARIANT = STM32F4xx/F405RGT_F415RGT

PORT = /dev/ttyACM0

OBJDIR = $(PWD)/obj

DFU = $(OBJDIR)/$(SKETCH).ino.dfu

HEX = $(OBJDIR)/$(SKETCH).ino.hex

ELF = $(OBJDIR)/$(SKETCH).ino.elf

HFLIB = ../../
LIB = ../../..

STM32DUINO = $(HOME)/.arduino15/packages/STMicroelectronics
STM32DVERSION = 2.3.0
GCCVERSION = 10.3.1-2.3

RECEIVER = ./obj/libraries/sbus/sbus.cpp.o

all: $(DFU)

$(DFU): $(HEX)
	$(HFLIB)/utils/dfuse-pack.py -i $(HEX) $(DFU)

$(HEX): $(ELF)
	objcopy -O ihex --set-start 0x8000000 $< $@

$(RUSTLIB): src/lib.rs
	cargo build

$(ELF): $(SKETCH).ino $(RUSTLIB)
	rm -rf /tmp/arduino-core-cache/
	arduino-cli compile --fqbn STMicroelectronics:stm32:Gen$(GEN):pnum=$(PNUM),usb=CDCgen,usb=CDCgen --libraries $(HFLIB),$(LIB) \
		--build-path $(OBJDIR) --warnings "all"
	mv /tmp/arduino-core-cache/*.a obj/core.a
	$(STM32DUINO)/tools/xpack-arm-none-eabi-gcc/$(GCCVERSION)/bin/arm-none-eabi-gcc \
	-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Os -DNDEBUG --specs=nano.specs \
	-Wl,--defsym=LD_FLASH_OFFSET=0 \
	-Wl,--defsym=LD_MAX_SIZE=1048576 \
	-Wl,--defsym=LD_MAX_DATA_SIZE=131072 \
	-Wl,--cref -Wl,--check-sections \
	-Wl,--gc-sections \
	-Wl,--entry=Reset_Handler \
	-Wl,--unresolved-symbols=report-all \
	-Wl,--warn-common \
	-Wl,--default-script=$(STM32DUINO)/hardware/stm32/$(STM32DVERSION)/variants/$(VARIANT)/ldscript.ld \
	-Wl,--script=$(STM32DUINO)/hardware/stm32/$(STM32DVERSION)/system/ldscript.ld \
	-Wl,-Map,./obj/$(SKETCH).ino.map \
	-L$(STM32DUINO)/tools/CMSIS/5.7.0/CMSIS/DSP/Lib/GCC/ \
	-larm_cortexM4lf_math -o ./obj/$(SKETCH).ino.elf \
	-L./obj -Wl,--start-group ./obj/sketch/$(SKETCH).ino.cpp.o \
	./obj/sketch/SrcWrapper.cpp.o \
	./obj/sketch/logic.cpp.o \
	./obj/libraries/SPI/SPI.cpp.o \
	./obj/libraries/SPI/utility/spi_com.c.o \
	$(RECEIVER) \
	./obj/libraries/SrcWrapper/HAL/*.o \
	./obj/libraries/SrcWrapper/HardwareTimer.cpp.o \
	./obj/libraries/SrcWrapper/LL/*.o \
	./obj/libraries/SrcWrapper/new.cpp.o \
	./obj/libraries/SrcWrapper/stm32/PortNames.c.o \
	./obj/libraries/SrcWrapper/stm32/analog.cpp.o \
	./obj/libraries/SrcWrapper/stm32/bootloader.c.o \
	./obj/libraries/SrcWrapper/stm32/clock.c.o \
	./obj/libraries/SrcWrapper/stm32/core_callback.c.o \
	./obj/libraries/SrcWrapper/stm32/dwt.c.o \
	./obj/libraries/SrcWrapper/stm32/hw_config.c.o \
	./obj/libraries/SrcWrapper/stm32/interrupt.cpp.o \
	./obj/libraries/SrcWrapper/stm32/otp.c.o \
	./obj/libraries/SrcWrapper/stm32/pinmap.c.o \
	./obj/libraries/SrcWrapper/stm32/stm32_def.c.o \
	./obj/libraries/SrcWrapper/stm32/system_stm32yyxx.c.o \
	./obj/libraries/SrcWrapper/stm32/timer.c.o \
	./obj/libraries/SrcWrapper/stm32/uart.c.o \
	./obj/libraries/SrcWrapper/syscalls.c.o \
	./obj/core/PeripheralPins.c.o \
	./obj/core/generic_clock.c.o \
	./obj/core/variant_generic.cpp.o \
	$(RUSTLIB) \
	./obj/core.a \
	-lc -Wl,--end-group -lm -lgcc -lstdc++

unbrick: $(DFU)
	dfu-util -a 0 -D $(DFU) -s :leave	

flash: $(DFU)
	echo -n 'R' > $(PORT)
	sleep 1
	dfu-util -a 0 -D $(DFU) -s :leave

clean:
	rm -rf obj/ target/

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200

