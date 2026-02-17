ROOT = ../../..

SIMSENSORS = $(ROOT)/simsensors/src

HF = ../..

HFSRC = $(HF)/src

HFWB = $(HF)/webots

world = $(HFWB)/worlds/twoexit.wbt

all: simtest

run: simtest
	./simtest

simtest: main.o
	g++ -o simtest main.o

main.o: main.cpp $(HFSRC)/*.hpp $(HFSRC)/pids/*.hpp \
	$(SIMSENSORS)/parsers/webots/* $(SIMSENSORS)/sensors/* 
	g++ -std=c++17 -Wall -c -I$(ROOT) -I$(HFSRC) main.cpp

edit:
	vim main.cpp

clean:
	rm -f simtest *.o *.csv

play:
	# ./playback.py $(world)
	./playback.py twowall.wbt
