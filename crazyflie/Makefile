SKETCH = $(shell basename "`pwd`")

PORT = /dev/ttyACM0

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

BIN = build/$(SKETCH).ino.bin

SRC = ../src

MSGS = ../src/__messages__.h

HASKDIR = ../haskell

DEPS = $(SKETCH).ino  *.cpp $(SRC)/*.h $(SRC)/*.hpp $(MSGS)

include ../snn_path.makefile

any:
	echo "Please specify cf2 or bolt"

bolt:  $(DEPS) bolt/*.cpp 
	cp bolt/motors.cpp ./__motors__.cpp
	arduino-cli compile \
		--build-property build.extra_flags=-DBOLT \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) $(SKETCH).ino

cf2: $(DEPS) cf2/*.cpp
	cp cf2/motors.cpp ./__motors__.cpp
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) $(SKETCH).ino

$(MSGS): ../msppg/messages.json
	cd ../msppg; make install; cd ../crazyflie
	cp ../msppg/messages.h $(MSGS)

haskell_config: copilot_core.cpp
	cp $(SRC)/pid/haskell/pid.hpp ./__pid__.hpp

lowres_config:
	cp $(SRC)/pid/lowres/pid.hpp ./__pid__.hpp

slowfast_config:
	cp $(SRC)/pid/slowfast/pid.hpp ./__pid__.hpp

standard_config:
	cp $(SRC)/pid/standard/pid.hpp ./__pid__.hpp


copilot_core.cpp : $(HASKDIR)/*.hs $(HASKDIR)/pids/*.hs
	runhaskell -i$(HASKDIR) -i$(HASKDIR)/pids $(HASKDIR)/Pids.hs -i$(PWD)
	mv copilot_core.c copilot_core.cpp

dfu: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D $(BIN)

clean:
	rm -rf build copilot* __*__.*

listen:
	miniterm.py $(PORT) 115200 --exit-char 3

edit:
	vim crazyflie.ino
