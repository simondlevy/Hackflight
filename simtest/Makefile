all: simtest

run: simtest
	./simtest

simtest: main.o
	g++ -o simtest main.o

simtest.o: main.cpp
	g++ -Wall -c main.cpp
