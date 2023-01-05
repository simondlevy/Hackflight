SKETCH = EspReceiver

FQBN = esp32:esp32:tinypico

PORT = /dev/ttyUSB1

all:
	arduino-cli compile --fqbn $(FQBN) . --warnings "all"

flash: 
	arduino-cli upload --port /dev/ttyUSB1 --fqbn $(FQBN)

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200

