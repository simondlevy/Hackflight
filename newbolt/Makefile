PORT = /dev/ttyACM0

SKETCH = newbolt

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

BIN = build/$(SKETCH).ino.bin

ROOT = $(HOME)/Desktop/hackflight

SRC = $(ROOT)/src

all : $(BIN)

$(BIN): $(SKETCH).ino $(EXTRA_SRC)
	arduino-cli compile --libraries \
		$(HOME)/Documents/Arduino/libraries --libraries $(ROOT)/../.. \
		--build-path ./build --fqbn $(FQBN) $(SKETCH).ino

flash: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D build/$(SKETCH).ino.bin

clean:
	rm -rf build

listen:
	miniterm.py $(PORT) 115200 --exit-char 3
