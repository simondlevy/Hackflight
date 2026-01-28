ROOT = ../..

SIMSENSORS = $(ROOT)/simsensors/src

world=../webots/worlds/flydar.wbt

all: simtest

run: simtest
	./simtest ../webots/protos/DiyQuad.proto $(world)

simtest: main.o
	g++ -o simtest main.o

main.o: main.cpp flydar.hpp \
	$(SIMSENSORS)/parsers/webots/* $(SIMSENSORS)/sensors/* 
	g++ -Wall -c -I$(ROOT) -I../src main.cpp

edit:
	vim main.cpp

clean:
	rm -f simtest *.o *.csv
