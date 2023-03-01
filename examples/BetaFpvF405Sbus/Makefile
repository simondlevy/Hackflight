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

