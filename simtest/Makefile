ROOT = ../..

SIMSENSORS = $(ROOT)/simsensors/src

world=../webots/worlds/twoexit.wbt

all: simtest

run: simtest
	./simtest ../webots/protos/DiyQuad.proto $(world)

simtest: main.o
	g++ -o simtest main.o

main.o: main.cpp ../src/*.hpp ../src/pids/*.hpp \
	$(SIMSENSORS)/parsers/webots/* $(SIMSENSORS)/sensors/* 
	g++ -Wall -std=c++17 -c -I$(ROOT) -I../src main.cpp

edit:
	vim main.cpp

clean:
	rm -f simtest *.o *.csv

play:
	# ./playback.py $(world)
	./playback.py twowall.wbt
