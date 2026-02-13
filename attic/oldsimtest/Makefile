ROOT = ../..

SIMSENSORS = $(ROOT)/simsensors/src

all: simtest

run: simtest
	./simtest ../webots/worlds/simple.wbt ../webots/protos/DiyQuad.proto 

simtest: main.o 
	g++ -o simtest main.o

main.o: main.cpp $(SIMSENSORS)/parsers/webots/* $(SIMSENSORS)/sensors/* 
	g++ -std=c++17 -Wall -c -I$(ROOT) -I../src main.cpp

edit:
	vim main.cpp

clean:
	rm -f simtest *.o
