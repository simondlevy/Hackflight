SKETCH = Teensy4

PORT = /dev/ttyACM0

BOARD = teensy:avr:teensy40

build:
	arduino-cli compile  --libraries ../..,../../.. --fqbn $(BOARD) $(SKETCH).ino

flash:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .

upload:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200
