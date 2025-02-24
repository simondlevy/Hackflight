FDIR = $(HOME)/Desktop/framework

NETWORK = difference_risp_train.txt

# IPADDR = 137.113.118.30
IPADDR = 10.42.0.1

PDIR =  $(FDIR)/processors/risp

LEVY_UTILS = $(FDIR)/include/utils/levy

INCLUDE = -I$(FDIR)/include -I$(LEVY_UTILS) -I$(FDIR)/cpp-apps/include -I$(PDIR)/include

FSRC = $(FDIR)/src

PSRC = $(PDIR)/src

OBJS = io_stream.o jspace.o network.o properties.o processor.o static_proc.o

ALL = rpi

all: $(ALL)

rpi: main.o $(OBJS)
	g++ -O3 -o rpi main.o $(OBJS)

main.o: main.cpp $(LEVY_UTILS)/*.hpp
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

run: rpi
	./rpi $(FDIR)/networks/$(NETWORK)

# Run this on the desktop/host
plot:
	$(HOME)/Desktop/spikeplotter/spikeplot.py -f $(HOME)/Desktop/framework/networks/$(NETWORK) \
		--server $(IPADDR) --ids 0,1,2 --display-counts

clean:
	rm -f $(ALL) *.o *.hpp

edit:
	vim main.cpp
