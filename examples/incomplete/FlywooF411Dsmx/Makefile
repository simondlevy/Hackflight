SKETCH = FlywooF411Dsmx

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F411CEUX,usb=CDCgen

PORT = /dev/ttyACM0

OBJ = $(PWD)/obj
HFLIB = ../../
LIB = ../../..
DFU = $(OBJ)/$(SKETCH).dfu 
HEX = $(OBJ)/$(SKETCH).ino.hex
SRC = $(HFLIB)/src

all: $(DFU)
all: $(HEX)

$(DFU): $(HEX)
	$(HFLIB)/utils/dfuse-pack.py -i $(HEX) $(DFU)

$(HEX): $(SKETCH).ino $(SRC)/*.h $(SRC)/*/*.h
	cp prebuild.sh \
		$(HOME)/.arduino15/packages/STMicroelectronics/hardware/stm32/2.5.0/system/extras
	arduino-cli compile --fqbn $(FQBN) --libraries $(HFLIB),$(LIB) --build-path $(OBJ)
	rm -f *.bin *.elf

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

