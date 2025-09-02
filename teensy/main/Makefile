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

FQBN = teensy:avr:teensy40

build: $(SKETCH).ino ../../src/msp/__messages__.h
	arduino-cli compile --libraries $(HOME)/Documents/Arduino/libraries --libraries ../../.. \
		--fqbn $(FQBN) $(SKETCH).ino

standard_config:
	cp ../../src/control/standard.hpp ../../src/__control__.hpp

../../src/msp/__messages__.h:
	cd ../../msppg; make install; cd ../teensy/main

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

listen:
	miniterm.py $(PORT) --exit-char 3

