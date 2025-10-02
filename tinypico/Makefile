FQBN = esp32:esp32:um_tinypico

PORT = /dev/ttyACM0

SRC = ../src

build: tinypico.ino *.cpp *.h $(SRC)/*.hpp $(SRC)/tasks/*.hpp \
	$(SRC)/msp/__messages__.h
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--fqbn $(FQBN) tinypico.ino

$(SRC)/msp/__messages__.h: ../msppg/messages.json
	cd ../msppg; make install; cd ../tinypico

standard_config:
	cp $(SRC)/control/standard.hpp $(SRC)/__control__.hpp

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN)

listen:
	miniterm.py $(PORT) 115200 --exit-char 3
