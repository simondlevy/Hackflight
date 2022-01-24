# Makefile for Esp32Now2Sbus transducer
#
# Copyright (c) 2022 Simon D. Levy
#
# MIT License

SKETCH = Esp32Now2Sbus
ROOT = ../../..

HDIR = $(ROOT)/haskell
CDIR = $(ROOT)/c
ADIR = $(ROOT)/arduino
CPPDIR = $(ADIR)/cpp

HSRC = $(SKETCH).hs \
       $(HDIR)/Receiver.hs \
       $(HDIR)/Demands.hs \
       $(HDIR)/Serial.hs \
       $(HDIR)/Messages.hs \
       $(HDIR)/Utils.hs \
	   $(CDIR)/comms.c


all: hackflight.cpp
	rm -rf $(SKETCH)
	mkdir $(SKETCH)
	cp hackflight.c $(SKETCH)/hackflight.cpp
	rm hackflight.c
	sed -i 's/extern/EXTERN/g' hackflight.h
	cat $(CDIR)/header.h hackflight.h > $(SKETCH)/hackflight.h
	rm hackflight.h
	cp $(ADIR)/Hackflight.ino $(SKETCH)/$(SKETCH).ino
	cp $(CDIR)/comms.c $(SKETCH)/comms.cpp
	cp $(CPPDIR)/arduino_debugger.hpp $(SKETCH)
	cp $(CPPDIR)/arduino_serial.cpp $(SKETCH)
	cp $(CPPDIR)/arduino_delay.cpp $(SKETCH)
	cp $(CPPDIR)/esp32_now.cpp $(SKETCH)
	cp $(CPPDIR)/esp32_now_receiver.cpp $(SKETCH)

hackflight.cpp: hackflight.c
	tail -n +8 hackflight.c > tmp.c
	cat $(CDIR)/header.c tmp.c > hackflight.c
	rm tmp.c

hackflight.c: $(HSRC) $(SKETCH).hs
	runhaskell -i$(HDIR) -i$(HDIR)/sensors -i$(HDIR)/pidcontrollers $(SKETCH).hs

clean:
	rm -rf $(SKETCH)
