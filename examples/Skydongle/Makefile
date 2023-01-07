SKETCH = Skydongle

FQBN = esp32:esp32:tinypico

PORT = /dev/ttyUSB1

HACKFLIGHT = ../../../Hackflight

all:
	arduino-cli compile --fqbn $(FQBN) . --warnings "all" --libraries $(HACKFLIGHT)

flash: 
	arduino-cli upload --port $(PORT) --fqbn $(FQBN)

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200

