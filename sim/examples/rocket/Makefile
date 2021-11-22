#
# Makefile for HackflightSim DJI Rocket example
#
# Copyright (c) 2021 on D. Levy
#
# MIT License

ROOT = ../../..
HDIR = $(ROOT)/haskell
CDIR = $(ROOT)/c

HSRC = Rocket.hs \
	   $(HDIR)/Dynamics.hs \
       $(HDIR)/HackflightSim.hs \
       $(HDIR)/Receiver.hs \
       $(HDIR)/Demands.hs \
       $(HDIR)/State.hs \
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
       $(HDIR)/dynamics/QuadXAP.hs


DSTDIR = ../../Source/HackflightSim/

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

hackflight.c: $(HSRC) Rocket.hs
	runhaskell -i$(HDIR) -i$(HDIR)/sensors -i$(HDIR)/pidcontrollers -i$(HDIR)/dynamics Rocket.hs

clean:
	rm -f hackflight.* tmp.c *~
