#
# Makefile for Hackflight sim version
#
# Copyright (c) 2021 on D. Levy
#
# MIT License

HSRC = Main.hs \
       HackflightSim.hs \
       Safety.hs \
       Receiver.hs \
       Hackflight.hs \
       Demands.hs \
       State.hs \
       Sensor.hs \
       Time.hs \
       Utils.hs \
       Mixer.hs \
       PidController.hs \
       pidcontrollers/AltHoldPid.hs \
       pidcontrollers/RatePid.hs \
       pidcontrollers/YawPid.hs \
       pidcontrollers/LevelPid.hs \
       sensors/Gyrometer.hs \
       sensors/Altimeter.hs \
       sensors/Quaternion.hs \
       sensors/OpticalFlow.hs

RUNH = runhaskell -isensors -ipidcontrollers Main.hs

DSTDIR = ../../Source/CopilotModule/

all: copilot.cpp copilot.h
	cp copilot.c $(DSTDIR)/copilot.cpp
	sed -i 's/extern/EXTERN/g' copilot.h
	cat cfiles/header.h copilot.h > $(DSTDIR)/copilot.h

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
	cat cfiles/header.c tmp.c > copilot.c
	rm tmp.c
