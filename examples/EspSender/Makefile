SKETCH = EspSender

FQBN = esp32:esp32:pico32

PORT = /dev/ttyUSB0

all:
	arduino-cli compile --fqbn $(FQBN) . --warnings "all"

flash: 
	arduino-cli upload --port $(PORT) --fqbn $(FQBN)

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200

