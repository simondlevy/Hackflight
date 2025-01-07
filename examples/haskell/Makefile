SKETCH = haskell

FQBN = teensy:avr:teensy40

PORT = /dev/ttyACM0

HASKDIR = ../../haskell

LIBS = $(HOME)/Documents/Arduino/libraries

build: $(SKETCH).ino copilot_core.cpp
	arduino-cli compile --libraries $(LIBS) --libraries ../.. --fqbn $(FQBN) $(SKETCH).ino

copilot_core.cpp : CoreTask.hs $(HASKDIR)/*.hs $(HASKDIR)/pids/*.hs
	runhaskell -i$(HASKDIR) -i$(HASKDIR)/pids CoreTask.hs
	mv copilot_core.c copilot_core.cpp

flash:
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) .

clean:
	rm -rf copilot*

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200
