all: main

main: main.o udp_sockets.o hackflight.o
	gcc -o main main.o udp_sockets.o hackflight.o

main.o: main.c udp_sockets.h hackflight.h
	gcc -Wall -std=c99 -c main.c

udp_sockets.o: udp_sockets.c udp_sockets.h
	gcc -Wall -std=c99 -c udp_sockets.c

hackflight.o: hackflight.c hackflight.h
	gcc -Wall -std=c99 -c hackflight.c

hackflight.c: Hackflight.hs Demands.hs VehicleState.hs
	runhaskell Hackflight.hs

hackflight.h: Hackflight.hs
	runhaskell Hackflight.hs

clean:
	rm -f main *.o hackflight.c hackflight.h *~

edit:
	vim main.c

run: main
	./main
