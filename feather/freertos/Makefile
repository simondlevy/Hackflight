SKETCH = freertos

FQBN = STMicroelectronics:stm32:GenF4:pnum=FEATHER_F405,usb=CDCgen

PORT = /dev/ttyACM0

LIBS = $(HOME)/Documents/Arduino/libraries

BIN = build/$(SKETCH).ino.bin

$(BIN): $(SKETCH).ino
	arduino-cli compile --libraries $(LIBS) --fqbn $(FQBN) --output-dir=./build $(SKETCH).ino

flash: $(BIN)
	dfu-util -a 0 -s 0x08000000:leave -D $(BIN)

clean:
	rm -rf build

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3 # exit on CTRL-C
