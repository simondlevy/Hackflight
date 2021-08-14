all: main

main: main.o udp_sockets.o
	gcc -o main main.o udp_sockets.o

main.o: main.c udp_sockets.h
	gcc -Wall -std=c99 -c main.c

udp_sockets.o: udp_sockets.c udp_sockets.h
	gcc -Wall -std=c99 -c udp_sockets.c

clean:
	rm -f main *~

edit:
	vim main.c

run: main
	./main
