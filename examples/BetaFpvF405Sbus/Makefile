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

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

PORT = /dev/ttyACM0

OBJDIR = $(PWD)/obj

DFU = $(OBJDIR)/$(SKETCH).ino.dfu

HEX = $(OBJDIR)/$(SKETCH).ino.hex

ELF = $(OBJDIR)/$(SKETCH).ino.elf

HFLIB = ../../
LIB = ../../..

all: $(DFU)

$(DFU): $(HEX)
	$(HFLIB)/utils/dfuse-pack.py -i $(HEX) $(DFU)

$(HEX): $(ELF)
	objcopy -O ihex --set-start 0x8000000 $< $@

$(ELF): $(SKETCH).ino
	rm -rf /tmp/arduino-core-cache/
	arduino-cli compile --fqbn $(FQBN) --libraries $(HFLIB),$(LIB) \
		--build-path $(OBJDIR) --warnings "all"
	mv /tmp/arduino-core-cache/*.a obj/core.a
	$(HOME)/.arduino15/packages/STMicroelectronics/tools/xpack-arm-none-eabi-gcc/10.3.1-2.3/bin/arm-none-eabi-gcc \
	-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Os -DNDEBUG --specs=nano.specs \
	-Wl,--defsym=LD_FLASH_OFFSET=0 \
	-Wl,--defsym=LD_MAX_SIZE=1048576 \
	-Wl,--defsym=LD_MAX_DATA_SIZE=131072 \
	-Wl,--cref -Wl,--check-sections \
	-Wl,--gc-sections \
	-Wl,--entry=Reset_Handler \
	-Wl,--unresolved-symbols=report-all \
	-Wl,--warn-common \
	-Wl,--default-script=$(HOME)/.arduino15/packages/STMicroelectronics/hardware/stm32/2.3.0/variants/STM32F4xx/F405RGT_F415RGT/ldscript.ld \
	-Wl,--script=$(HOME)/.arduino15/packages/STMicroelectronics/hardware/stm32/2.3.0/system/ldscript.ld \
	-Wl,-Map,./obj/$SKETCH.ino.map \
	-L$(HOME)/.arduino15/packages/STMicroelectronics/tools/CMSIS/5.7.0/CMSIS/DSP/Lib/GCC/ \
	-larm_cortexM4lf_math -o ./obj/$(SKETCH).ino.elf \
	-L./obj -Wl,--start-group ./obj/sketch/$(SKETCH).ino.cpp.o \
	./obj/sketch/SrcWrapper.cpp.o \
	./obj/sketch/logic.cpp.o \
	./obj/libraries/SPI/SPI.cpp.o \
	./obj/libraries/SPI/utility/spi_com.c.o \
	./obj/libraries/sbus/sbus.cpp.o \
	./obj/libraries/SrcWrapper/HAL/*.c.o \
	./obj/libraries/SrcWrapper/HAL/stm32yyxx_hal_xspi.c.o \
	./obj/libraries/SrcWrapper/HardwareTimer.cpp.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_adc.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_bdma.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_comp.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_cordic.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_crc.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_crs.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_dac.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_delayblock.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_dlyb.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_dma.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_dma2d.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_exti.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_fmac.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_fmc.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_fmpi2c.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_fsmc.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_gpio.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_hrtim.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_i2c.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_icache.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_lpgpio.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_lptim.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_lpuart.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_mdma.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_opamp.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_pka.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_pwr.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_rcc.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_rng.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_rtc.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_sdmmc.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_spi.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_swpmi.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_tim.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_ucpd.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_usart.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_usb.c.o \
	./obj/libraries/SrcWrapper/LL/stm32yyxx_ll_utils.c.o \
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
	./obj/core/PeripheralPins_FEATHER_F405.c.o \
	./obj/core/generic_clock.c.o \
	./obj/core/variant_FEATHER_F405.cpp.o \
	./obj/core/variant_generic.cpp.o \
	./obj/core.a \
	-lc -Wl,--end-group -lm -lgcc -lstdc++

unbrick: $(DFU)
	dfu-util -a 0 -D $(DFU) -s :leave	

flash: $(DFU)
	echo -n 'R' > $(PORT)
	sleep 1
	dfu-util -a 0 -D $(DFU) -s :leave

clean:
	rm -rf obj

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200

