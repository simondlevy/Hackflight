SKETCH = $(shell basename "`pwd`")

PORT = /dev/ttyACM0

FQBN = teensy:avr:teensy40

SRC = ../src

MSGS = ../src/__messages__.h

include ../snn_path.makefile

build: $(SKETCH).ino  *.cpp *.h $(SRC)/*.hpp $(MSGS)
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--fqbn $(FQBN) $(SKETCH).ino

$(MSGS): ../msppg/messages.json
	cd ../msppg; make install; cd ../crazyflie
	cp ../msppg/messages.h $(MSGS)

lowres_config:
	cp $(SRC)/pid/lowres/pid.hpp ./__pid__.hpp

nopos_config:
	cp $(SRC)/pid/nopos/pid.hpp ./__pid__.hpp

split_config:
	cp $(SRC)/pid/split/pid.hpp ./__pid__.hpp

standard_config:
	cp $(SRC)/pid/standard/pid.hpp $(SRC)/__pid__.hpp

snn_config:
	cp $(SRC)/control/snn.hpp ./__pid__.hpp
	cp $(EMBDIR)/examples/difference/*.hpp .

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .
clean:
	rm -rf build copilot* ../src/__*__.*

listen:
	miniterm.py $(PORT) 115200 --exit-char 3
