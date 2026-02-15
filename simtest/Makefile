ROOT = ../..

SIMSENSORS = $(ROOT)/simsensors/src

world = ../webots/worlds/twoexit.wbt

log = "log.csv"

all: simtest

run: simtest
	./simtest ../webots/protos/DiyQuad.proto $(world) $(log)

simtest: main.o
	g++ -o simtest main.o

main.o: main.cpp ../src/*.hpp ../src/pids/*.hpp \
	$(SIMSENSORS)/parsers/webots/* $(SIMSENSORS)/sensors/* 
	g++ -std=c++17 -Wall -c -I$(ROOT) -I../src main.cpp

edit:
	vim main.cpp

clean:
	rm -f simtest *.o *.csv

play:
	# ./playback.py $(world)
	./playback.py twowall.wbt
