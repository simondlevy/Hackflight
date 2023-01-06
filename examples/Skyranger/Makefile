SKETCH = Skyranger

FQBN = esp32:esp32:pico32

PORT = /dev/ttyUSB0

VL53L5 = ../../../VL53L5
PAA3905 = ../../../PAA3905
HACKFLIGHT = ../../../Hackflight

all:
	@echo "This doesn't work yet"
	arduino-cli compile --fqbn $(FQBN) . --warnings "all" \
    --libraries $(VL53L5) --libraries $(PAA3905) --libraries $(HACKFLIGHT)
	

flash: 
	arduino-cli upload --port $(PORT) --fqbn $(FQBN)

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200

