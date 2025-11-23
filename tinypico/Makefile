SKETCH = $(shell basename "`pwd`")

FQBN = esp32:esp32:um_tinypico

PORT = /dev/ttyACM0

MSGS = ../src/__messages__.h

build: $(SKETCH).ino $(MSGS)
	arduino-cli compile \
		--libraries $(HOME)/Documents/Arduino/libraries \
		--libraries ../.. \
		--fqbn $(FQBN) $(SKETCH).ino

$(MSGS): ../msppg/messages.json
	cd ../msppg; make install; cd ../tinypico
	cp ../msppg/messages.h $(MSGS)

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN)

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3

clean:
	rm -rf ../src/__*__.*

