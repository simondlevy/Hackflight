SKETCH = $(shell basename "`pwd`")

ROOT = $(HOME)/Desktop/

FQBN = esp32:esp32:um_omgs3

PORT = /dev/ttyACM0

LIBS = $(HOME)/Documents/Arduino/libraries

build: $(SKETCH).ino
	arduino-cli compile --libraries $(LIBS) --libraries $(ROOT) \
		--fqbn $(FQBN) $(SKETCH).ino

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

clean:
	rm -rf /tmp/arduino/

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3 # exit on CTRL-C
