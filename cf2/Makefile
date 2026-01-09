SKETCH = $(shell basename "`pwd`")

PORT = /dev/ttyACM0

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

BIN = build/$(SKETCH).ino.bin

SRC = ../src

MSGS = ../src/__messages__.h

DEPS = $(SKETCH).ino  *.cpp $(SRC)/*.h $(SRC)/*.hpp $(MSGS)

include ../snn_path.makefile

any:
	echo "Please specify cf2 or bolt"

bolt: $(DEPS) bolt11/*.* 
	cp bolt11/motors.cpp ./__motors__.cpp
	cp bolt11/uart_pins.h ./__uart_pins__.h
	arduino-cli compile \
		--build-property build.extra_flags=-DBOLT \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) $(SKETCH).ino

cf2: $(DEPS) cf2/*.*
	cp cf2/motors.cpp ./__motors__.cpp
	cp cf2/uart_pins.h ./__uart_pins__.h
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) $(SKETCH).ino

big: $(DEPS) biggy/*.*
	cp biggy/motors.cpp ./__motors__.cpp
	cp biggy/uart_pins.h ./__uart_pins__.h
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) $(SKETCH).ino

$(MSGS): ../msppg/messages.json
	cd ../msppg; make install; cd ../cf2
	cp ../msppg/messages.h $(MSGS)

lowres_config:
	cp $(SRC)/pid/lowres/pid.hpp ./__pid__.hpp

slowfast_config:
	cp $(SRC)/pid/slowfast/pid.hpp ./__pid__.hpp

standard_config:
	cp $(SRC)/pid/standard/pid.hpp ./__pid__.hpp


dfu: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D $(BIN)

clean:
	rm -rf build copilot* __*__.*

listen:
	miniterm.py $(PORT) 115200 --exit-char 3

edit:
	vim cf2.ino
