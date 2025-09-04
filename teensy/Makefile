SKETCH = teensy

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

ROOT = ..

build: $(SKETCH).ino $(ROOT)/src/msp/__messages__.h
	arduino-cli compile --libraries \
		$(HOME)/Documents/Arduino/libraries --libraries $(ROOT)/.. \
		--fqbn $(FQBN) $(SKETCH).ino

standard_config:
	cp $(ROOT)/src/control/standard.hpp $(ROOT)/src/__control__.hpp

$(ROOT)/src/msp/__messages__.h:
	cd $(ROOT)/msppg; make install; cd $(ROOT)/teensy

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

listen:
	miniterm.py $(PORT) --exit-char 3

