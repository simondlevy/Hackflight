PORT = /dev/ttyACM0

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

BIN = build/weact.ino.bin

SRC = ../src

all: weact.ino  
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) weact.ino

flash: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D build/weact.ino.bin

clean:
	rm -rf build

edit:
	vim weact.ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3


