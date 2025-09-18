PORT = /dev/ttyACM0

SKETCH = newbolt

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

BIN = build/$(SKETCH).ino.bin

HFDIR = $(PWD)/..

all : $(BIN)

$(BIN): $(SKETCH).ino *.cpp 
	arduino-cli compile --libraries \
		$(HOME)/Documents/Arduino/libraries --libraries ../.. \
		--build-path ./build --fqbn $(FQBN) $(SKETCH).ino

flash: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D build/$(SKETCH).ino.bin

clean:
	rm -rf build

listen:
	miniterm.py $(PORT) 115200 --exit-char 3

standard_config:
	cp $(HFDIR)/src/control/standard.hpp $(HFDIR)/src/__control__.hpp
	cd $(HFDIR)/msppg; make install; cd ..


