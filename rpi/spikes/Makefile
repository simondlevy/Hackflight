NETWORK = difference_risp_train.txt

ROOT = $(HOME)/Desktop

FDIR = $(ROOT)/framework

PDIR =  $(FDIR)/processors/risp

LEVY_UTILS = $(FDIR)/include/utils/levy

INCLUDE = -I$(ROOT) -I$(FDIR)/include -I$(LEVY_UTILS) -I$(FDIR)/cpp-apps/include -I$(PDIR)/include

FSRC = $(FDIR)/src

PSRC = $(PDIR)/src

OBJS = io_stream.o jspace.o network.o properties.o processor.o static_proc.o

ALL = spikes

all: $(ALL)

spikes: main.o $(OBJS)
	g++ -O3 -o spikes main.o $(OBJS)

main.o: main.cpp $(LEVY_UTILS)/*.hpp $(ROOT)/posix-utils/*.hpp \
	../../src/msp/parser.hpp ../../src/msp/serializer.hpp 
	g++ $(INCLUDE) -I../src -O3 -Wall -c main.cpp

io_stream.o: $(FSRC)/io_stream.cpp
	g++ $(INCLUDE) -O3 -Wall -c $(FSRC)/io_stream.cpp

jspace.o: $(FSRC)/jspace.cpp
	g++ $(INCLUDE) -O3 -Wall -c $(FSRC)/jspace.cpp

network.o: $(FSRC)/network.cpp
	g++ $(INCLUDE) -O3 -Wall -c $(FSRC)/network.cpp

properties.o: $(FSRC)/properties.cpp
	g++ $(INCLUDE) -O3 -Wall -c $(FSRC)/properties.cpp

processor.o: $(PSRC)/processor.cpp
	g++ $(INCLUDE) -O3 -Wall -c $(PSRC)/processor.cpp

static_proc.o: $(PSRC)/static_proc.cpp
	g++ $(INCLUDE) -O3 -Wall -c $(PSRC)/static_proc.cpp

run: spikes
	./spikes $(FDIR)/networks/$(NETWORK)

clean:
	rm -f $(ALL) *.o *.hpp

edit:
	vim main.cpp
