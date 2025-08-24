SKETCH = bmi088

# FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen
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

listen:
	miniterm.py $(PORT) 115200 --exit-char 3 # exit on CTRL-C
	# plink -serial $(PORT)

