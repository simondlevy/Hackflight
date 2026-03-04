SKETCH = $(shell basename "`pwd`")

ROOT = ../..

PORT = /dev/ttyACM0

FQBN = STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen

BIN = build/$(SKETCH).ino.bin

SRC = $(ROOT)/src

MSGS = $(SRC)/msp/__messages__.h

HASKDIR = $(ROOT)/haskell

include ../../snn_path.makefile

DEPS = $(SKETCH).ino  *.cpp *.h $(SRC)/*.hpp $(SRC)/firmware/*.hpp \
	   $(SRC)/firmware/*/*.hpp $(MSGS) 

COMPILE = arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--build-path ./build \
		--fqbn $(FQBN) $(SKETCH).ino

any:
	@echo Please specify bolt or cf2 or nomotors

cf2: $(DEPS)
	$(COMPILE)

bolt: $(DEPS)
	$(COMPILE) --build-property "compiler.cpp.extra_flags=\"-DBOLT\""

nomotors: $(DEPS)
	$(COMPILE) --build-property "compiler.cpp.extra_flags=\"-DNOMOTORS\""

$(MSGS): $(ROOT)/msppg/messages.json
	cd $(ROOT)/msppg; make install; cd ../examples/$(SKETCH)

dfu: $(BIN)
	dfu-util -d 0483:5740 -a 0 -s 0x08000000:leave -D build/$(SKETCH).ino.bin

clean:
	rm -rf build $(MSGS)

listen:
	miniterm.py $(PORT) 115200 --exit-char 3

edit:
	vim $(SKETCH).ino
