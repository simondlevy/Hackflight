#
# Makefile for Hackflight Copilot socket version
#
# Copyright (c) 2021 on D. Levy
#
# MIT License

HSRC = Main.hs \
	   Hackflight.hs \
	   Receiver.hs \
	   Demands.hs \
	   State.hs \
	   Sensor.hs \
	   Utils.hs \
	   Mixer.hs \
	   PidController.hs \
	   pidcontrollers/RatePid.hs \
	   pidcontrollers/YawPid.hs \
	   pidcontrollers/AltHoldPid.hs \
	   pidcontrollers/LevelPid.hs \
	   pidcontrollers/PosHoldPid.hs \
	   sensors/Gyrometer.hs \
	   sensors/Altimeter.hs \
	   sensors/Quaternion.hs \
	   sensors/OpticalFlow.hs

RUNH = runhaskell -isensors -ipidcontrollers Main.hs

TOP = /mnt/d/Users/levys

SIM = $(TOP)/Documents/Unreal\ Projects/MulticopterSim/Source/CopilotModule

ARDUINO = $(TOP)/Documents/Arduino/libraries/Hackflight/examples/Copilot

all: hackflight.h hackflight.c

hackflight.c: $(HSRC)
	$(RUNH)
	
hackflight.h: $(HSRC)
	$(RUNH)

clean:
	rm -f tmp.c hackflight.c hackflight.h *~

commit:
	git commit -a -m="More Haskell Copilot work"

copy: hackflight.cpp hackflight.h
	cp hackflight.c $(SIM)/hackflight.cpp
	cp hackflight.h $(SIM)
	cp hackflight.c $(ARDUINO)/hackflight.cpp
	cp hackflight.h $(ARDUINO)

hackflight.cpp: hackflight.c
	tail -n +8 hackflight.c > tmp.c
	cat header.c tmp.c > hackflight.c
	rm tmp.c
