SKETCH = betafpv

PORT = /dev/ttyACM0

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

BIN = build/$(SKETCH).ino.bin

DFU = build/$(SKETCH).ino.dfu

HEX = build/$(SKETCH).ino.hex

SRC = ../src

all: $(BIN)

dfu: $(DFU)

$(DFU): $(HEX)
	./dfuse-pack.py -i $(HEX) $(DFU)

$(HEX): $(SKETCH).ino
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) $(SKETCH).ino

$(BIN): $(SKETCH).ino
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) $(SKETCH).ino

unbrick: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D build/$(SKETCH).ino.bin

clean:
	rm -rf build

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3


