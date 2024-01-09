SKETCH = Teensy4

FQBN = teensy:avr:teensy40

PORT = /dev/ttyACM0

build:
	cp FreeRTOSConfig_hackflight.h ../../../FreeRTOS-Teensy4/src/FreeRTOSConfig_default.h
	arduino-cli compile  --libraries ../..,../../.. --fqbn $(FQBN)

upload:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200
