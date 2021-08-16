all: socket_main

socket_main: socket_main.o udp_sockets.o hackflight.o
	gcc -o socket_main socket_main.o udp_sockets.o hackflight.o -lm

socket_main.o: socket_main.c udp_sockets.h hackflight.h
	gcc -Wall -std=c99 -c socket_main.c

udp_sockets.o: udp_sockets.c udp_sockets.h
	gcc -Wall -std=c99 -c udp_sockets.c

hackflight.o: hackflight.c hackflight.h
	gcc -Wall -std=c99 -c hackflight.c

hackflight.c: Hackflight.hs Demands.hs VehicleState.hs Gyrometer.hs
	runhaskell Hackflight.hs

hackflight.h: Hackflight.hs Demands.hs VehicleState.hs Gyrometer.hs
	runhaskell Hackflight.hs

clean:
	rm -f socket_main *.o hackflight.c hackflight.h *~

edit:
	vim socket_main.c

run: socket_main
	./socket_main

commit:
	git commit -a -m="More Haskell Copilot work"
