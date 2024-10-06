SKETCH = master

FQBN = esp32:esp32:um_tinypico

PORT = /dev/ttyUSB0

LIBS = $(HOME)/Documents/Arduino/libraries

build: $(SKETCH).ino
	arduino-cli compile --libraries $(LIBS) --libraries ../../.. --fqbn $(FQBN) $(SKETCH).ino

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

clean:
	rm -rf obj

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200
