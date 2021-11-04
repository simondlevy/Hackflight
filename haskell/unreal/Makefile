#
# Makefile for Hackflight sim version
#
# Copyright (c) 2021 on D. Levy
#
# MIT License

HSRC = Main.hs \
       Hackflight.hs \
       ../Receiver.hs \
       Hackflight.hs \
       ../Demands.hs \
       State.hs \
       ../Sensor.hs \
       ../Time.hs \
       ../Utils.hs \
       Mixer.hs \
       PidController.hs \
       pidcontrollers/AltHoldPid.hs \
       pidcontrollers/RatePid.hs \
       pidcontrollers/YawPid.hs \
       pidcontrollers/LevelPid.hs \
       ../sensors/Gyrometer.hs \
       ../sensors/Quaternion.hs \
       ../sensors/Altimeter.hs \
       ../sensors/OpticalFlow.hs

RUNH = runhaskell -i.. -i../sensors -ipidcontrollers Main.hs

DSTDIR = ../../unreal/Source/CopilotModule/

all: hackflight.cpp hackflight.h
	cp hackflight.c $(DSTDIR)/hackflight.cpp
	sed -i 's/extern/EXTERN/g' hackflight.h
	cat cfiles/header.h hackflight.h > $(DSTDIR)/hackflight.h

hackflight.c: $(HSRC) Main.hs
	$(RUNH)

hackflight.h: $(HSRC) Main.hs
	$(RUNH)

clean:
	rm -f hackflight.* tmp.c *~

commit:
	git commit -a -m="More Haskell Copilot work"

hackflight.cpp: hackflight.c
	tail -n +8 hackflight.c > tmp.c
	cat cfiles/header.c tmp.c > hackflight.c
	rm tmp.c
