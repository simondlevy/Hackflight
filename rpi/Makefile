NETWORK = difference_risp_train.txt

ROOT = $(HOME)

INCLUDE = -I$(ROOT)

FDIR = $(ROOT)/framework

PDIR =  $(FDIR)/processors/risp

INCLUDE = -I$(ROOT) -I$(FDIR)/include -I$(LEVY_UTILS) -I$(FDIR)/cpp-apps/include -I$(PDIR)/include

LEVY_UTILS = $(FDIR)/include/utils/levy

standard: btserver.cpp $(ROOT)/posix-utils/*.hpp ../src/msp/*.hpp ../src/msp/messages.h
	g++ $(INCLUDE) -I../src -O3 -Wall btserver.cpp -o btserver

snn: btserver.cpp $(ROOT)/posix-utils/*.hpp ../src/msp/*.hpp ../src/msp/messages.h
	g++ $(INCLUDE) -I../src -O3 -Wall btserver.cpp -o btserver

../src/msp/messages.h: ../msppg/messages.json
	cd ../msppg; make install; cd ../rpi

run: btserver
	./btserver

clean:
	rm -f $(ALL) *.o

edit:
	vim btserver.cpp
