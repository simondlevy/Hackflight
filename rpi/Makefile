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

ROOT = $(HOME)

EDIR = $(ROOT)/embedded-neuromorphic/levy/encoder-decoder

FDIR = $(ROOT)/framework

PDIR =  $(FDIR)/processors/risp

NETWORK = difference_risp_train.txt

INCLUDE = -I$(ROOT) -I../src -I$(FDIR)/include -I$(FDIR)/include/utils/levy -I$(FDIR)/cpp-apps/include

standard: standard.cpp $(ROOT)/posix-utils/*.hpp ../src/msp/*.hpp ../src/msp/messages.h
	g++ -Wall $(INCLUDE) -O3 -Wall standard.cpp -o btserver

# Just encoder for now (full monty would use libframework.a)
snn: snn.cpp $(ROOT)/posix-utils/*.hpp ../src/msp/*.hpp ../src/msp/messages.h difference_risp_train.hpp
	g++ -Wall $(INCLUDE) -I$(EDIR)/src -I../src -O3 -Wall snn.cpp -o btserver

new: newsnn.cpp
	g++ -Wall $(INCLUDE) -O3 newsnn.cpp $(FDIR)/lib/libframework.a -o btserver

difference_risp_train.hpp: 
	$(EDIR)/compiler.py $(FDIR)/networks/$(NETWORK)

../src/msp/messages.h: ../msppg/messages.json
	cd ../msppg; make install; cd ../rpi

run: btserver
	./btserver

clean:
	rm -f btserver *.o *.hpp
