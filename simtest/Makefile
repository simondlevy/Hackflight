ROOT = ../..

SIMSENSORS = $(ROOT)/simsensors/src

world=../webots/worlds/twoexit.wbt

all: simtest

run: simtest
	./simtest ../webots/protos/DiyQuad.proto $(world)

simtest: main.o twoexit.o
	g++ -o simtest main.o twoexit.o

main.o: main.cpp *.hpp ../src/*.hpp ../src/pids/*.hpp \
	$(SIMSENSORS)/parsers/webots/* $(SIMSENSORS)/sensors/* 
	g++ -Wall -c -I$(ROOT) -I../src main.cpp

twoexit.o: twoexit.cpp *.hpp
	g++ -Wall -c -I$(ROOT) -I../src twoexit.cpp

edit:
	vim main.cpp

clean:
	rm -f simtest *.o *.csv

play:
	./playlog.py
