PORT = /dev/ttyACM0

SKETCH = cf2

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

BIN = build/$(SKETCH).ino.bin

SRC = ../src

all : $(BIN)

$(BIN): $(SKETCH).ino  *.cpp *.h $(SRC)/*.hpp $(SRC)/tasks/*.hpp $(SRC)/msp/__messages__.h
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) $(SKETCH).ino

$(SRC)/msp/__messages__.h: ../msppg/messages.json
	cd ../msppg; make install; cd ../$(SKETCH)

standard_config:
	cp $(SRC)/control/standard.hpp $(SRC)/__control__.hpp

flash: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D build/$(SKETCH).ino.bin

clean:
	rm -rf build

listen:
	miniterm.py $(PORT) 115200 --exit-char 3


