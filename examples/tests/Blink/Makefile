FQBN = teensy:avr:teensy40

SKETCH = Blink

PORT = /dev/ttyACM0

build: $(SKETCH).ino 
	arduino-cli compile --fqbn $(FQBN) $(SKETCH).ino

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

edit:
	vim $(SKETCH).ino
