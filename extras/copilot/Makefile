all: main

main: main.c sockets/posix_sockets.h sockets/udp_socket.h
	gcc -Wall -std=c99 main.c -o main

clean:
	rm -f main *~

edit:
	vim main.c
