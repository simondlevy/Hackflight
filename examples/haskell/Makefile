SKETCH = haskell

BOARD = teensy:avr:teensy40

PORT = /dev/ttyACM0

HASKDIR = ../../haskell

LIBS = $(HOME)/Documents/Arduino/libraries

build: $(SKETCH).ino copilot_core.cpp
	arduino-cli compile --libraries $(LIBS) --libraries ../../.. --fqbn $(BOARD) $(SKETCH).ino

copilot_core.cpp : $(HASKDIR)/*.hs $(HASKDIR)/pids/*.hs
	runhaskell -i$(HASKDIR) -i$(HASKDIR)/pids CoreTaskReal.hs
	mv copilot_core.c copilot_core.cpp

flash:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .

clean:
	rm -rf obj

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200
