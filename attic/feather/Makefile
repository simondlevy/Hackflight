SKETCH = feather

PORT = /dev/ttyACM0

FQBN = STMicroelectronics:stm32:GenF4:pnum=FEATHER_F405,usb=CDCgen

SRC = ../src

MSGS = $(SRC)/msp/__messages__.h

BIN = build/$(SKETCH).ino.bin

HEX = build/$(SKETCH).ino.hex

$(BIN): $(HEX)

$(HEX):  $(SKETCH).ino *.cpp $(SRC)/*.hpp $(SRC)/tasks/*.hpp $(MSGS)
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries --libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) $(SKETCH).ino


$(MSGS): ../msppg/messages.json
	cd ../msppg; make install; cd ../$(SKETCH)

flash: $(BIN)
	echo -n 'R' > $(PORT) &
	sleep 2
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D $(BIN)

unbrick: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D $(BIN)

standard_config:
	cp $(SRC)/control/standard.hpp $(SRC)/__control__.hpp

clean:
	rm -rf build __*__.cpp $(MSGS)

listen:
	miniterm.py $(PORT) 115200 --exit-char 3

edit:
	vim $(SKETCH).ino
