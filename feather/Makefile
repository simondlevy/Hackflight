SKETCH = feather

FQBN = STMicroelectronics:stm32:GenF4:pnum=FEATHER_F405,usb=CDCgen

PORT = /dev/ttyACM0

LIBS = $(HOME)/Documents/Arduino/libraries

BIN = build/$(SKETCH).ino.bin

$(BIN): $(SKETCH).ino
	arduino-cli compile --libraries $(LIBS) --fqbn $(FQBN) --output-dir=./build $(SKETCH).ino

flash: $(BIN)
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

unbrick: $(BIN)
	sh $(HOME)/.arduino15/packages/STMicroelectronics/tools/STM32Tools/2.2.1/stm32CubeProg.sh 2 $(BIN) 0x0

clean:
	rm -rf obj

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3 # exit on CTRL-C
