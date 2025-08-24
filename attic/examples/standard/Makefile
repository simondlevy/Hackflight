SKETCH = standard

include ../makefile.include

SRC = ../../src

build:
	cp FreeRTOSConfig.h $(SRC)
	arduino-cli compile  --libraries ../../.. --libraries $(HOME)/Documents/Arduino/libraries \
		--fqbn $(FQBN) $(SKETCH).ino
	rm ../../src/FreeRTOSConfig.h

config:
	cp $(SRC)/control/standard.hpp $(SRC)/control.hpp
	cd ../../msppg; make install; cd ..
