PORT = /dev/ttyACM0

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

BIN = build/diy.ino.bin

SRC = ../src

bolt: diy.ino  *.cpp *.h $(SRC)/*.hpp $(SRC)/tasks/*.hpp \
	$(SRC)/msp/__messages__.h
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) diy.ino

$(SRC)/msp/__messages__.h: ../msppg/messages.json
	cd ../msppg; make install; cd ../diy

standard_config:
	cp $(SRC)/control/standard.hpp $(SRC)/__control__.hpp

flash: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D build/diy.ino.bin

clean:
	rm -rf build __*__.cpp

listen:
	miniterm.py $(PORT) 115200 --exit-char 3


