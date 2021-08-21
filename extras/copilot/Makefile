#
# Makefile for Hackflight Copilot socket version
#
# Copyright (c) 2021 on D. Levy
#
# MIT License

HSRC = Main.hs Hackflight.hs Receiver.hs Demands.hs VehicleState.hs Sensor.hs \
	   Utils.hs Mixer.hs PidController.hs \
	   pidcontrollers/YawPid.hs pidcontrollers/AltHoldPid.hs \
	   sensors/Gyrometer.hs sensors/Altimeter.hs sensors/Euler.hs

RUNH = runhaskell -isensors -ipidcontrollers Main.hs

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
	
hackflight.h: $(HSRC)
	$(RUNH)

clean:
	rm -f main *.o hackflight.c hackflight.h *~

edit:
	vim socket_main.c

run: main
	./main

commit:
	git commit -a -m="More Haskell Copilot work"
