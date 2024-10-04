SKETCH = dongle

# For complete list run arduino-cli board listall
BOARD = esp32:esp32:m5stack_atom
# BOARD = esp32:esp32:um_tinypico

PORT = /dev/ttyUSB0

LIBS = $(HOME)/Documents/Arduino/libraries

build: $(SKETCH).ino
	arduino-cli compile --libraries $(LIBS) --fqbn $(BOARD) $(SKETCH).ino

flash:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200
