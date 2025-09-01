SKETCH = main

ifeq ($(OS),Windows_NT)
PORT = COM3
TERM = plink -serial
PYTHON = python
else
PORT = /dev/ttyACM0
TERM = miniterm.py
PYTHON = python3
endif

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

build: $(SKETCH).ino ../../src/msp/messages.h
	arduino-cli compile \
	--libraries $(HOME)/Documents/Arduino/libraries --libraries ../.. \
	--fqbn $(FQBN) $(SKETCH).ino --build-path ./build

standard_config:
	cp ../../src/control/standard.hpp ../../src/__control__.hpp

../../src/msp/messages.h:
	cd ../../msppg; make install; cd ../examples/main

flash:
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D build/$(SKETCH).ino.bin

clean:
	rm -rf build

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) --exit-char 3
