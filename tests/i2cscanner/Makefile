SKETCH = i2cscanner

PORT = /dev/ttyACM0

# FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen
FQBN = STMicroelectronics:stm32:GenF4:pnum=FEATHER_F405,usb=CDCgen

BIN = build/$(SKETCH).ino.bin

HEX = build/$(SKETCH).ino.hex

$(DFU): $(HEX)
	../utils/dfuse-pack.py -i $(HEX) $(DFU)

$(HEX):  $(SKETCH).ino
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) $(SKETCH).ino

flash: $(DFU)
	echo -n 'R' > $(PORT)
	sleep 2
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D $(BIN)

# Building .hex will also build .bin
unbrick: $(HEX)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D $(BIN)

clean:
	rm -rf build __*__.cpp

listen:
	miniterm.py $(PORT) 115200 --exit-char 3

edit:
	vim $(SKETCH).ino
