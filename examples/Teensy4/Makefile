SKETCH = Teensy4

FQBN = teensy:avr:teensy40

PORT = /dev/ttyACM0

build:
	arduino-cli compile  --libraries ../..,../../.. --fqbn $(FQBN)

upload:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200
