ROOT = ../..

SIMSENSORS = $(ROOT)/simsensors/src

HF = ..

HFSRC = $(HF)/src

HFWB = $(HF)/webots

all: twoexit

twoexit: twoexit.o
	g++ -o twoexit twoexit.o

twoexit.o: twoexit.cpp $(HFSRC)/*.hpp $(HFSRC)/pids/*.hpp  $(HFSRC)/simulator/*.hpp \
	$(SIMSENSORS)/parsers/webots/* $(SIMSENSORS)/sensors/* 
	g++ -std=c++17 -Wall -c -I$(ROOT) -I$(HFSRC) twoexit.cpp

clean:
	rm -f twoexit *.o *.csv
