ROOT = ../..

SIMSENSORS = $(ROOT)/simsensors/src

HF = ..

HFSRC = $(HF)/src

HFWB = $(HF)/webots

ALL = twoexit pingpong 

DEPS = $(HFSRC)/*.hpp $(HFSRC)/pids/*.hpp  $(HFSRC)/simulator/*.hpp \
	   $(SIMSENSORS)/parsers/webots/* $(SIMSENSORS)/sensors/* 

CC = g++ -std=c++17 -Wall -c -I$(ROOT) -I$(HFSRC)

all: $(ALL)

twoexit: twoexit.o
	g++ -o twoexit twoexit.o

pingpong: pingpong.o
	g++ -o pingpong pingpong.o

twoexit.o: twoexit.cpp $(DEPS)
	$(CC) twoexit.cpp

pingpong.o: $(DEPS)
	$(CC) pingpong.cpp

clean:
	rm -f $(ALL) *.o *.csv
