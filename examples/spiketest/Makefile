SKETCH = spiketest

FDIR = $(HOME)/Desktop/framework

EDIR = $(HOME)/Desktop/embedded-neuromorphic/arduino/encoder-decoder

NETWORK = difference_risp_train

# FQBN = teensy:avr:teensy40
FQBN = teensy:avr:teensy41

PORT = /dev/ttyACM0

LIBS = $(HOME)/Documents/Arduino/libraries

build: $(SKETCH).ino $(NETWORK).hpp 
	arduino-cli compile --libraries $(LIBS) --libraries $(EDIR) --libraries ../../.. \
		--fqbn $(FQBN) $(SKETCH).ino

$(NETWORK).hpp:
	$(EDIR)/compiler.py $(FDIR)/networks/$(NETWORK).txt

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

clean:
	rm *.hpp

edit:
	vim $(SKETCH).ino

listen:
	miniterm.py $(PORT) 115200 --exit-char 3 # exit on CTRL-C
