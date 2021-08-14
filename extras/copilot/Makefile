all: main

main: main.c sockets/*.h
	gcc -Wall -std=c99 main.c -o main

clean:
	rm -f main *~

edit:
	vim main.c

run: main
	./main
