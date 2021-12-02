# Makefile for Ladbyug Flight Controller with DSMX receiver
#
# Copyright (c) 2021 Simon D. Levy
#
# MIT License

ROOT = ../../..
HDIR = $(ROOT)/haskell
CDIR = $(ROOT)/c

HSRC = TinyPico.hs \
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


DSTDIR = ../../Hackflight
CPPDIR = ../../cpp

all: hackflight.cpp
	cp hackflight.c $(DSTDIR)/hackflight.cpp
	rm hackflight.c
	sed -i 's/extern/EXTERN/g' hackflight.h
	cat $(CDIR)/header.h hackflight.h > $(DSTDIR)/hackflight.h
	rm hackflight.h
	cp $(CDIR)/serial.c $(DSTDIR)/serial.cpp
	cp $(CPPDIR)/esp32_dsmrx.cpp $(DSTDIR)
	cp $(CPPDIR)/tinypico_led.cpp $(DSTDIR)
	cp $(CPPDIR)/arduino_i2c.cpp $(DSTDIR)
	cp $(CPPDIR)/arduino_serial.cpp $(DSTDIR)
	cp $(CPPDIR)/arduino_time.cpp $(DSTDIR)
	cp $(CPPDIR)/arduino_usfs_nogyro.cpp $(DSTDIR)

hackflight.cpp: hackflight.c
	tail -n +8 hackflight.c > tmp.c
	cat $(CDIR)/header.c tmp.c > hackflight.c
	rm tmp.c

hackflight.c: $(HSRC) TinyPico.hs
	runhaskell -i$(HDIR) -i$(HDIR)/sensors -i$(HDIR)/pidcontrollers TinyPico.hs

clean:
	rm -f $(DSTDIR)/*.h $(DSTDIR)/*.cpp
