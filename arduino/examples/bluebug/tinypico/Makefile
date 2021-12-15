# Makefile for Ladbyug Flight Controller with DSMX receiver
#
# Copyright (c) 2021 Simon D. Levy
#
# MIT License

SKETCH = TinyPICO
ROOT = ../../../..
HDIR = $(ROOT)/haskell
CDIR = $(ROOT)/c
CPPDIR = $(ROOT)/arduino/cpp

HSRC = TinyPICO.hs \
       $(HDIR)/Parser.hs \
       $(HDIR)/Serial.hs \
       $(HDIR)/Time.hs \
       $(HDIR)/Utils.hs

all: hackflight.cpp
	mkdir -p $(SKETCH)
	cp hackflight.c $(SKETCH)/hackflight.cpp
	rm hackflight.c
	sed -i 's/extern/EXTERN/g' hackflight.h
	cat $(CDIR)/header.h hackflight.h > $(SKETCH)/hackflight.h
	rm hackflight.h
	cp ../../Hackflight.ino $(SKETCH)/$(SKETCH).ino
	cp $(CDIR)/serial.c $(SKETCH)/serial.cpp
	cp $(CPPDIR)/tinypico_led.cpp $(SKETCH)
	cp $(CPPDIR)/arduino_serial.cpp $(SKETCH)
	cp $(CPPDIR)/arduino_time.cpp $(SKETCH)

hackflight.cpp: hackflight.c
	tail -n +8 hackflight.c > tmp.c
	cat $(CDIR)/header.c tmp.c > hackflight.c
	rm tmp.c

hackflight.c: $(HSRC) TinyPICO.hs
	runhaskell -i$(HDIR) -i$(HDIR)/sensors -i$(HDIR)/pidcontrollers TinyPICO.hs

clean:
	rm -f $(SKETCH)/*.h $(SKETCH)/*.cpp
