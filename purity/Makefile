test: main.o
	g++ -o test main.o

main.o: main.cpp
	g++ -Wall -I../src -c main.cpp

edit:
	vim main.cpp

clean:
	rm -rf test main.o
