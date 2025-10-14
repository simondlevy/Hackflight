SKETCH = feather

PORT = /dev/ttyACM0

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

SRC = ../src

BIN = build/$(SKETCH).ino.bin

DFU = build/$(SKETCH).ino.dfu

HEX = build/$(SKETCH).ino.hex

$(DFU): $(HEX)
	../utils/dfuse-pack.py -i $(HEX) $(DFU)

$(HEX):  *.cpp *.h $(SRC)/*.hpp $(SRC)/tasks/*.hpp \
	$(SRC)/msp/__messages__.h
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) $(SKETCH).ino

$(SRC)/msp/__messages__.h: ../msppg/messages.json
	cd ../msppg; make install; cd ../$(SKETCH)

standard_config:
	cp $(SRC)/control/standard.hpp $(SRC)/__control__.hpp

flash: $(DFU)
	echo -n 'R' > $(PORT)
	sleep 2
	dfu-util -a 0 -D $(DFU) -s :leave

# Building .hex will also build .bin
unbrick: $(HEX)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D $(BIN)

clean:
	rm -rf build __*__.cpp

listen:
	miniterm.py $(PORT) 115200 --exit-char 3

edit:
	vim $(SKETCH).ino
