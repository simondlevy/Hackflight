SKETCH = EspReceiver

FQBN = esp32:esp32:tinypico

PORT = /dev/ttyUSB1

all:
	arduino-cli compile --fqbn esp32:esp32:tinypico .

flash: 
	arduino-cli upload -p /dev/ttyUSB1 --fqbn esp32:esp32:tinypico

clean:
	rm -rf obj

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200

