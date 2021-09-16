#
# Makefile for Hackflight sim version
#
# Copyright (c) 2021 on D. Levy
#
# MIT License

SRCDIR = $(MYHOME)/Documents/Hackflight/source

HSRC = Main.hs \
	   $(SRCDIR)/Safety.hs \
	   $(SRCDIR)/Serial.hs \
       $(SRCDIR)/Receiver.hs \
       $(SRCDIR)/Hackflight.hs \
       $(SRCDIR)/Demands.hs \
       $(SRCDIR)/State.hs \
       $(SRCDIR)/Sensor.hs \
       $(SRCDIR)/Time.hs \
       $(SRCDIR)/Utils.hs \
       $(SRCDIR)/Mixer.hs \
       $(SRCDIR)/PidController.hs \
       $(SRCDIR)/pidcontrollers/RatePid.hs \
       $(SRCDIR)/pidcontrollers/YawPid.hs \
       $(SRCDIR)/pidcontrollers/AltHoldPid.hs \
       $(SRCDIR)/pidcontrollers/LevelPid.hs \
       $(SRCDIR)/pidcontrollers/PosHoldPid.hs \
       $(SRCDIR)/sensors/Gyrometer.hs \
       $(SRCDIR)/sensors/Altimeter.hs \
       $(SRCDIR)/sensors/Quaternion.hs \
       $(SRCDIR)/sensors/OpticalFlow.hs

RUNH = runhaskell -i$(SRCDIR) -i$(SRCDIR)/sensors -i$(SRCDIR)/pidcontrollers Main.hs

DST = ../Source/HackflightSim

all: copilot.cpp copilot.h
	cp copilot.c $(DST)/copilot.cpp
	sed -i 's/extern/EXTERN/g' copilot.h
	cat $(SRCDIR)/cfiles/header.h copilot.h > $(DST)/copilot.h

copilot.c: $(HSRC) Main.hs
	$(RUNH)

copilot.h: $(HSRC) Main.hs
	$(RUNH)

clean:
	rm -f copilot.* tmp.c *~

commit:
	git commit -a -m="More Haskell Copilot work"

copilot.cpp: copilot.c
	tail -n +8 copilot.c > tmp.c
	cat $(SRCDIR)/cfiles/header.c tmp.c > copilot.c
	rm tmp.c
