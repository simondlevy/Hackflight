SKETCH = vl53l1

PORT = /dev/ttyACM0

BOARD = teensy:avr:teensy40

LIBS = $(HOME)/Documents/Arduino/libraries

build: $(SKETCH).ino 
	arduino-cli compile  --libraries $(LIBS) --fqbn $(BOARD) $(SKETCH).ino

flash:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .

upload:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3 # exit on CTRL-C
