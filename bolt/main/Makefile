SKETCH = main

BIN = build/$(SKETCH).ino.bin

all: $(BIN)

PORT = /dev/ttyACM0

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

build/$(SKETCH).ino.bin: $(SKETCH).ino vapplication.c *.hpp *.cpp ../../src/msp/messages.h
	arduino-cli compile --libraries $(HOME)/Documents/Arduino/libraries --libraries ../../.. \
		--fqbn $(FQBN) $(SKETCH).ino --build-path ./build

standard_config:
	cp ../../src/control/standard.hpp ../../src/__control__.hpp

../../src/msp/messages.h:
	cd ../../msppg; make install; cd ../newbolt/main

flash: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D build/$(SKETCH).ino.bin

clean:
	rm -rf build

listen:
	miniterm.py $(PORT) --exit-char 3

