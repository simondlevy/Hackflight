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

