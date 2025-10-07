SKETCH = proxy

FQBN = teensy:avr:teensy40

PORT = /dev/ttyACM0

SRC = ../src

build: $(SKETCH).ino $(SRC)/*.hpp $(SRC)/tasks/*.hpp $(SRC)/msp/__messages__.h
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--fqbn $(FQBN) $(SKETCH).ino


$(SRC)/msp/__messages__.h: ../msppg/messages.json
	cd ../msppg; make install; cd ../crazyflie

standard_config:
	cp $(SRC)/control/standard.hpp $(SRC)/__control__.hpp

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

clean:
	rm -rf obj

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3
