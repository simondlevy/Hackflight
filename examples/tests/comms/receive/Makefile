SKETCH = receive

FQBN = teensy:avr:teensy40

PORT = /dev/ttyACM0
#PORT = COM3

LIBS = $(HOME)/Documents/Arduino/libraries

SRC = ../../../src/

build:
	arduino-cli compile  --libraries ../../.. --libraries $(HOME)/Documents/Arduino/libraries \
		--fqbn $(FQBN) $(SKETCH).ino


flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN)

edit:
	vim $(SKETCH).ino

