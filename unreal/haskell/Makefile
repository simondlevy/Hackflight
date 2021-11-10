#
# Makefile for Hackflight sim version
#
# Copyright (c) 2021 on D. Levy
#
# MIT License

ROOT = ../..
HDIR = $(ROOT)/haskell
CDIR = $(ROOT)/c

HSRC = Main.hs \
	   Dynamics.hs \
       $(HDIR)/Hackflight.hs \
       $(HDIR)/Receiver.hs \
       $(HDIR)/Demands.hs \
       $(HDIR)/State.hs \
       $(HDIR)/Sensor.hs \
       $(HDIR)/Time.hs \
       $(HDIR)/Utils.hs \
       $(HDIR)/Mixers.hs \
       $(HDIR)/Motors.hs \
       $(HDIR)/PidController.hs \
       $(HDIR)/pidcontrollers/AltHoldPid.hs \
       $(HDIR)/pidcontrollers/YawPid.hs \
       $(HDIR)/pidcontrollers/RatePid.hs \
       $(HDIR)/pidcontrollers/LevelPid.hs \
       $(HDIR)/pidcontrollers/PosHoldPid.hs \
       $(HDIR)/sensors/Gyrometer.hs \
       $(HDIR)/sensors/Quaternion.hs \
       $(HDIR)/sensors/Altimeter.hs \
       $(HDIR)/sensors/OpticalFlow.hs


DSTDIR = ../Source/HackflightSim/

all: hackflight.cpp
	cp hackflight.c $(DSTDIR)/hackflight.cpp
	rm hackflight.c
	sed -i 's/extern/EXTERN/g' hackflight.h
	cat $(CDIR)/header.h hackflight.h > $(DSTDIR)/hackflight.h
	rm hackflight.h

hackflight.cpp: hackflight.c
	tail -n +8 hackflight.c > tmp.c
	cat $(CDIR)/header.c tmp.c > hackflight.c
	rm tmp.c

hackflight.c: $(HSRC) Main.hs
	runhaskell -i$(HDIR) -i$(HDIR)/sensors -i$(HDIR)/pidcontrollers Main.hs

clean:
	rm -f hackflight.* tmp.c *~
