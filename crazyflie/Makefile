SKETCH = $(shell basename "`pwd`")

PORT = /dev/ttyACM0

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

BIN = build/$(SKETCH).ino.bin

SRC = ../src

MSGS = ../src/__messages__.h

HASKDIR = ../haskell

include ../snn_path.makefile

any:
	echo "Please specify cf2 or bolt"

bolt: $(SKETCH).ino  *.cpp *.h bolt/*.cpp $(SRC)/*.hpp $(MSGS)
	cp bolt/motors.cpp ./__motors__.cpp
	arduino-cli compile \
		--build-property build.extra_flags=-DBOLT \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) $(SKETCH).ino

cf2: $(SKETCH).ino  *.cpp *.h cf2/*.cpp $(SRC)/*.hpp $(MSGS)
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
	cp $(SRC)/control/haskell.hpp ../src/__control__.hpp

lowres_config:
	cp $(SRC)/control/lowres.hpp ../src/__control__.hpp

nopos_config:
	cp $(SRC)/control/nopos.hpp ../src/__control__.hpp

split_config:
	cp $(SRC)/control/split.hpp ../src/__control__.hpp

standard_config:
	cp $(SRC)/control/standard.hpp ../src/__control__.hpp

snn_config:
	cp $(SRC)/control/snn.hpp ../src/__control__.hpp
	cp $(EMBDIR)/examples/difference/*.hpp .

copilot_core.cpp : $(HASKDIR)/*.hs $(HASKDIR)/pids/*.hs
	runhaskell -i$(HASKDIR) -i$(HASKDIR)/pids $(HASKDIR)/Pids.hs -i$(PWD)
	mv copilot_core.c copilot_core.cpp

dfu: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D $(BIN)

clean:
	rm -rf build copilot* ../src/__*__.*

listen:
	miniterm.py $(PORT) 115200 --exit-char 3
