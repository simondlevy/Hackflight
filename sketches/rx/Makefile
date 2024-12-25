SKETCH = rx

# For complete list run arduino-cli board listall
# FQBN = esp32:esp32:um_tinypico
FQBN =esp32:esp32:um_tinys3

PORT = /dev/ttyACM1

LIBS = $(HOME)/Documents/Arduino/libraries 

build: $(SKETCH).ino
	arduino-cli compile --libraries $(LIBS) --libraries ../.. --fqbn $(FQBN) $(SKETCH).ino

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200
