SKETCH = TestTransmitter

PORT = /dev/ttyUSB0

LIB = $(HOME)/Documents/Arduino/libraries

FQBN = esp32:esp32:tinypico

build:
	arduino-cli compile  --libraries $(LIB),.. --fqbn $(FQBN) $(SKETCH).ino

upload:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

listen:
	miniterm $(PORT) 115200 --exit-char 3 # exit on CTRL-C

edit:
	vim $(SKETCH).ino
