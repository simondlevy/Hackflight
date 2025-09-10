SKETCH = bolt

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

BIN = build/$(SKETCH).ino.bin

ROOT = ..

all : $(BIN)

$(BIN): $(SKETCH).ino vapplication.c *.cpp *.h
	arduino-cli compile --libraries \
		$(HOME)/Documents/Arduino/libraries --libraries $(ROOT)/.. \
		--build-path ./build --fqbn $(FQBN) $(SKETCH).ino

standard_config:
	cp $(ROOT)/src/control/standard.hpp $(ROOT)/src/__control__.hpp

$(ROOT)/src/msp/__messages__.h:
	cd $(ROOT)/msppg; make install; cd $(ROOT)/teensy

flash: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D build/$(SKETCH).ino.bin

clean:
	rm -rf build

listen:
	miniterm.py $(PORT) 115200 --exit-char 3

