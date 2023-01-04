SKETCH = EspReceiver

FQBN = esp32:esp32:tinypico

PORT = /dev/ttyUSB1

all:
	arduino-cli compile --fqbn $(FQBN) .

flash: 
	arduino-cli upload -p /dev/ttyUSB1 --fqbn $(FQBN)

clean:
	rm -rf obj

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200

