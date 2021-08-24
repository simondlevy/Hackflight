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

all: main

main: socket_main.o udp_sockets.o hackflight.o
	gcc -o main socket_main.o udp_sockets.o hackflight.o -lm

socket_main.o: socket_main.c udp_sockets.h hackflight.h
	gcc -Wall -std=c99 -c socket_main.c

udp_sockets.o: udp_sockets.c udp_sockets.h
	gcc -Wall -std=c99 -c udp_sockets.c

hackflight.o: hackflight.c hackflight.h
	gcc -Wall -std=c99 -c hackflight.c

hackflight.c: $(HSRC)
	$(RUNH)
	tail -n +8 hackflight.c > tmp.c
	cat header.c tmp.c > hackflight.c
	rm tmp.c
	
hackflight.h: $(HSRC)
	$(RUNH)

clean:
	rm -f main *.o tmp.c hackflight.c hackflight.h *~

edit:
	vim socket_main.c

run: main
	./main

commit:
	git commit -a -m="More Haskell Copilot work"

copy: hackflight.c hackflight.h
	cp hackflight.c $(SIM)/hackflight.cpp
	cp hackflight.h $(SIM)
