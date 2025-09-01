PORT = /dev/ttyACM0

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

BIN = build/led.ino.bin

all: $(BIN)

build/led.ino.bin: led.ino
	arduino-cli compile --libraries $(HOME)/Documents/Arduino/libraries \
		--fqbn $(FQBN) led.ino --build-path ./build

flash: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D build/led.ino.bin

clean:
	rm -rf build/*

listen:
	miniterm.py $(PORT) --exit-char 3

