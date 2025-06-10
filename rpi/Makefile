#  Makefile for Bluetooth server running on Raspberry Pi
#
#  Copyright (C) 2025 Simon D. Levy
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http:--www.gnu.org/licenses/>.

ROOT = $(HOME)/Desktop

EDIR = $(ROOT)/embedded-neuromorphic/levy/encoder-decoder

FDIR = $(ROOT)/framework

PDIR =  $(FDIR)/processors/risp

NETWORK = difference_risp_train.txt

INCLUDE = -I$(ROOT) -I../src

FINCLUDE = -I$(FDIR)/include -I$(FDIR)/include/utils/levy -I$(FDIR)/cpp-apps/include -I$(PDIR)/include

standard: standard.cpp $(ROOT)/posix-utils/*.hpp ../src/msp/*.hpp ../src/msp/messages.h
	g++ -Wall $(INCLUDE) -O3 -Wall standard.cpp -o btserver

snn: snn.cpp $(ROOT)/posix-utils/*.hpp ../src/msp/*.hpp ../src/msp/messages.h difference_risp_train.hpp
	g++ -Wall $(INCLUDE) -I$(EDIR)/src -I../src -O3 -Wall snn.cpp -o btserver

new: newsnn.o 
	g++ -Wall -o btserver -O3 newsnn.o $(FDIR)/obj/io_stream.o \
		$(FDIR)/lib/libframework.a $(PDIR)/lib/libproc1.a $(PDIR)/lib/libproc2.a

newsnn.o: newsnn.cpp
	g++ -Wall $(INCLUDE) $(FINCLUDE) -c -O3 newsnn.cpp

test: testsnn.o 
	g++ -Wall -o btserver -O3 testsnn.o $(FDIR)/obj/io_stream.o \
		$(FDIR)/lib/libframework.a $(PDIR)/lib/libproc1.a $(PDIR)/lib/libproc2.a

testsnn.o: testsnn.cpp
	g++ -Wall $(INCLUDE) $(FINCLUDE) -c -O3 testsnn.cpp

difference_risp_train.hpp: 
	$(EDIR)/compiler.py $(FDIR)/networks/$(NETWORK)

../src/msp/messages.h: ../msppg/messages.json
	cd ../msppg; make install; cd ../rpi

run: btserver
	./btserver

clean:
	rm -f btserver *.o
