SKETCH = btserver

ifeq ($(OS),Windows_NT)
PORT = COM3
TERM = plink -serial
PYTHON = python
else
PORT = /dev/ttyACM0
TERM = miniterm.py
PYTHON = python3
endif

FQBN = esp32:esp32:um_tinypico

build: $(SKETCH).ino 
	arduino-cli compile --libraries $(HOME)/Documents/Arduino/libraries \
		--fqbn $(FQBN) $(SKETCH).ino

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

listen:
	miniterm.py $(PORT) 115200 --exit-char 3

