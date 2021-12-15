# Makefile for Ladbyug Flight Controller with DSMX receiver
#
# Copyright (c) 2021 Simon D. Levy
#
# MIT License

ROOT = ../../../..

HDIR = $(ROOT)/haskell
CDIR = $(ROOT)/c
CPPDIR = $(ROOT)/arduino/cpp
DSTDIR = TinyPICO

HSRC = TinyPICO.hs \
       $(HDIR)/Parser.hs \
       $(HDIR)/Serial.hs \
       $(HDIR)/Time.hs \
       $(HDIR)/Utils.hs


all: hackflight.cpp
	cp hackflight.c $(DSTDIR)/hackflight.cpp
	rm hackflight.c
	sed -i 's/extern/EXTERN/g' hackflight.h
	cat $(CDIR)/header.h hackflight.h > $(DSTDIR)/hackflight.h
	rm hackflight.h
	cp $(CDIR)/serial.c $(DSTDIR)/serial.cpp
	cp $(CPPDIR)/tinypico_led.cpp $(DSTDIR)
	cp $(CPPDIR)/arduino_serial.cpp $(DSTDIR)
	cp $(CPPDIR)/arduino_time.cpp $(DSTDIR)

hackflight.cpp: hackflight.c
	tail -n +8 hackflight.c > tmp.c
	cat $(CDIR)/header.c tmp.c > hackflight.c
	rm tmp.c

hackflight.c: $(HSRC) TinyPICO.hs
	runhaskell -i$(HDIR) -i$(HDIR)/sensors -i$(HDIR)/pidcontrollers TinyPICO.hs

clean:
	rm -f $(DSTDIR)/*.h $(DSTDIR)/*.cpp
