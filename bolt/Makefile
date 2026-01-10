SKETCH = $(shell basename "`pwd`")

PORT = /dev/ttyACM0

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

BIN = build/$(SKETCH).ino.bin

SRC = ../src

MSGS = $(SRC)/msp/__messages__.h

HASKDIR = ../haskell

include ../snn_path.makefile

bolt: $(SKETCH).ino  *.cpp *.h $(SRC)/*.hpp $(SRC)/tasks/*.hpp $(MSGS)
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) $(SKETCH).ino

$(MSGS): ../msppg/messages.json
	cd ../msppg; make install; cd ../bolt

haskell_config: copilot_core.cpp
	cp $(SRC)/control/haskell.hpp ./__control__.hpp

copilot_core.cpp : $(HASKDIR)/*.hs $(HASKDIR)/pids/*.hs
	runhaskell -i$(HASKDIR) -i$(HASKDIR)/pids $(HASKDIR)/Pids.hs -i$(PWD)
	mv copilot_core.c copilot_core.cpp

standard_config:
	cp $(SRC)/control/standard.hpp ./__control__.hpp

snn_config:
	cp $(SRC)/control/snn.hpp ./__control__.hpp
	cp $(EMBDIR)/examples/difference/*.hpp .

dfu: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D build/bolt.ino.bin

clean:
	rm -rf build __control__.hpp $(MSGS)

listen:
	miniterm.py $(PORT) 115200 --exit-char 3


