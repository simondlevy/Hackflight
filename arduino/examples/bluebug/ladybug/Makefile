# Makefile for Ladbyug Flight Controller with DSMX receiver
#
# Copyright (c) 2021 Simon D. Levy
#
# MIT License

SKETCH = Ladybug
ROOT = ../../../..

HDIR = $(ROOT)/haskell
CDIR = $(ROOT)/c
ADIR = $(ROOT)/arduino
CPPDIR = $(ADIR)/cpp

HSRC = Ladybug.hs \
       $(HDIR)/HackflightFull.hs \
       $(HDIR)/Receiver.hs \
       $(HDIR)/Demands.hs \
       $(HDIR)/PidController.hs \
       $(HDIR)/Safety.hs \
       $(HDIR)/Parser.hs \
       $(HDIR)/Serial.hs \
       $(HDIR)/Messages.hs \
       $(HDIR)/Time.hs \
       $(HDIR)/Mixers.hs \
       $(HDIR)/Motors.hs \
       $(HDIR)/Utils.hs \
	   $(HDIR)/sensors/Gyrometer.hs \
	   $(HDIR)/sensors/Quaternion.hs \
       $(HDIR)/pidcontrollers/RatePid.hs \
       $(HDIR)/pidcontrollers/YawPid.hs \
       $(HDIR)/pidcontrollers/LevelPid.hs \
	   $(CDIR)/motors.c \
	   $(CDIR)/serial.c

all: hackflight.cpp
	rm -rf $(SKETCH)
	mkdir -p $(SKETCH)
	cp hackflight.c $(SKETCH)/hackflight.cpp
	rm hackflight.c
	sed -i 's/extern/EXTERN/g' hackflight.h
	cat $(CDIR)/header.h hackflight.h > $(SKETCH)/hackflight.h
	rm hackflight.h
	cp $(ADIR)/Hackflight.ino $(SKETCH)/$(SKETCH).ino
	cp $(CDIR)/serial.c $(SKETCH)/serial.cpp
	cp $(CPPDIR)/arduino_serial.cpp $(SKETCH)
	cp $(CPPDIR)/arduino_serial1.cpp $(SKETCH)
	cp $(CPPDIR)/arduino_debug.cpp $(SKETCH)

hackflight.cpp: hackflight.c
	tail -n +8 hackflight.c > tmp.c
	cat $(CDIR)/header.c tmp.c > hackflight.c
	rm tmp.c

hackflight.c: $(HSRC) Ladybug.hs
	runhaskell -i$(HDIR) -i$(HDIR)/sensors -i$(HDIR)/pidcontrollers Ladybug.hs

clean:
	rm -f $(SKETCH)/*.h $(SKETCH)/*.cpp
