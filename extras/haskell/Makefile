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
	   VehicleState.hs \
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

SIM = /mnt/d/Users/levys/Documents/Unreal\ Projects/MulticopterSim/Source/CopilotModule

all: hackflight.h hackflight.c

hackflight.c: $(HSRC)
	$(RUNH)
	tail -n +8 hackflight.c > tmp.c
	cat header.c tmp.c > hackflight.c
	rm tmp.c
	
hackflight.h: $(HSRC)
	$(RUNH)

clean:
	rm -f tmp.c hackflight.c hackflight.h *~

commit:
	git commit -a -m="More Haskell Copilot work"

copy: hackflight.c hackflight.h
	cp hackflight.c $(SIM)/hackflight.cpp
	cp hackflight.h $(SIM)
