#
# Makefile for Hackflight Copilot socket version
#
# Copyright (c) 2021 on D. Levy
#
# MIT License

HSRC = Main.hs \
       ../Hackflight.hs \
	   ../Receiver.hs \
	   ../Demands.hs \
	   ../State.hs \
	   ../Sensor.hs \
	   ../Utils.hs \
	   ../Mixer.hs \
	   ../PidController.hs \
	   ../pidcontrollers/RatePid.hs \
	   ../pidcontrollers/YawPid.hs \
	   ../pidcontrollers/AltHoldPid.hs \
	   ../pidcontrollers/LevelPid.hs \
	   ../pidcontrollers/PosHoldPid.hs \
	   ../sensors/Gyrometer.hs \
	   ../sensors/Altimeter.hs \
	   ../sensors/Quaternion.hs \
	   ../sensors/OpticalFlow.hs

RUNH = runhaskell -i.. -i../sensors -i../pidcontrollers Main.hs

DST = /mnt/d/Users/levys/Documents/Arduino/libraries/Hackflight/examples/CopilotFull/

all: copilot.h copilot.c

copilot.c: $(HSRC) Main.hs
	$(RUNH)

copilot.h: $(HSRC) Main.hs
	$(RUNH)

clean:
	rm -f *.c *.h *~

commit:
	git commit -a -m="More Haskell Copilot work"

copy: copilot.cpp copilot.h
	cp copilot.c $(DST)/copilot.cpp
	cp copilot.h $(DST)/copilot.h

copilot.cpp: copilot.c
	tail -n +8 copilot.c > tmp.c
	cat ../header.c tmp.c > copilot.c
	rm tmp.c
