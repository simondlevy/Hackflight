SKETCH = standard

FQBN = teensy:avr:teensy40

PORT = /dev/ttyACM0

LIBS = $(HOME)/Documents/Arduino/libraries

SRC = ../../src/

build:
	cp FreeRTOSConfig.h $(SRC)
	arduino-cli compile  --libraries ../../.. --libraries $(HOME)/Documents/Arduino/libraries \
		--fqbn $(FQBN) $(SKETCH).ino
	rm ../../src/FreeRTOSConfig.h

config:
	cp $(SRC)/control/standard.hpp $(SRC)/control.hpp
	cd ../../msppg; make install; cd ..

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3 # exit on CTRL-C

