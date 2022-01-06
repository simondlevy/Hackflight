# Makefile for Ladbyug Flight Controller with DSMX receiver
#
# Copyright (c) 2021 Simon D. Levy
#
# MIT License

SKETCH = Dsmx2Esp32Now
ROOT = ../../..

HDIR = $(ROOT)/haskell
CDIR = $(ROOT)/c
ADIR = $(ROOT)/arduino
CPPDIR = $(ADIR)/cpp

HSRC = $(SKETCH).hs \
       $(HDIR)/HackflightProxy.hs \
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
	   $(CDIR)/serial.c


all: hackflight.cpp
	rm -rf $(SKETCH)
	mkdir $(SKETCH)
	cp hackflight.c $(SKETCH)/hackflight.cpp
	rm hackflight.c
	sed -i 's/extern/EXTERN/g' hackflight.h
	cat $(CDIR)/header.h hackflight.h > $(SKETCH)/hackflight.h
	rm hackflight.h
	cp $(ADIR)/Hackflight.ino $(SKETCH)/$(SKETCH).ino
	cp $(CPPDIR)/esp32_dsmrx.cpp $(SKETCH)
	cp $(CPPDIR)/esp32_sbus_out.cpp $(SKETCH)
	cp $(CPPDIR)/arduino_debugger.hpp $(SKETCH)
	cp $(CPPDIR)/arduino_serial.cpp $(SKETCH)
	cp tmp.cpp $(SKETCH)

hackflight.cpp: hackflight.c
	tail -n +8 hackflight.c > tmp.c
	cat $(CDIR)/header.c tmp.c > hackflight.c
	rm tmp.c

hackflight.c: $(HSRC) $(SKETCH).hs
	runhaskell -i$(HDIR) -i$(HDIR)/sensors -i$(HDIR)/pidcontrollers $(SKETCH).hs

clean:
	rm -rf $(SKETCH)
