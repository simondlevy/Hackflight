SKETCH = xiaoc6

FQBN = esp32:esp32:XIAO_ESP32C6

PORT = /dev/ttyACM0

LIBS = $(HOME)/Documents/Arduino/libraries

BIN = build/$(SKETCH).ino.bin

$(BIN): $(SKETCH).ino
	arduino-cli compile --libraries $(LIBS) --fqbn $(FQBN) --output-dir=./build $(SKETCH).ino

flash: $(BIN)
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

clean:
	rm -rf build

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3 # exit on CTRL-C

