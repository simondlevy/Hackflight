SKETCH = new

FQBN = teensy:avr:teensy40

PORT = /dev/ttyACM0

LIBS = $(HOME)/Documents/Arduino/libraries

build: $(SKETCH).ino
	# arduino-cli compile --libraries $(LIBS) --libraries ../../.. --fqbn $(FQBN) $(SKETCH).ino
	arduino-cli compile --libraries $(LIBS) --libraries ../.. --fqbn $(FQBN) $(SKETCH).ino

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

clean:
	rm -rf obj

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200
