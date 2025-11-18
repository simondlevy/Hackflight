SKETCH = $(shell basename "`pwd`")

PORT = /dev/ttyACM0

FQBN = teensy:avr:teensy40

SRC = ../src

MSGS = ../src/__messages__.h

HASKDIR = ../haskell

include ../snn_path.makefile

build: $(SKETCH).ino  *.cpp *.h $(SRC)/*.hpp $(MSGS)
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
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

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .
clean:
	rm -rf build copilot* ../src/__*__.*

listen:
	miniterm.py $(PORT) 115200 --exit-char 3
