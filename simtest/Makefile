ROOT = ../..

SIMSENSORS = $(ROOT)/simsensors/src

world=../webots/worlds/maze.wbt

all: simtest

run: simtest
	./simtest ../webots/protos/DiyQuad.proto $(world)

simtest: main.o 
	g++ -o simtest main.o

main.o: main.cpp $(SIMSENSORS)/parsers/webots/* $(SIMSENSORS)/sensors/* 
	g++ -Wall -c -I$(ROOT) -I../src main.cpp

edit:
	vim main.cpp

clean:
	rm -f simtest *.o
