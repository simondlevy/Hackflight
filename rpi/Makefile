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

# TeNNLab framework stuff ----------------------------------------------------

ROOT = $(HOME)

EDIR = $(ROOT)/embedded-neuromorphic/levy/encoder-decoder

FDIR = $(ROOT)/framework

NETWORK = difference_risp_train.txt

INCLUDE = -I$(ROOT) -I../src

standard: btserver.cpp $(ROOT)/posix-utils/*.hpp ../src/msp/*.hpp ../src/msp/messages.h
	g++ -Wall $(INCLUDE) -O3 -Wall btserver.cpp -o btserver

# Just encoder for now (full monty would use libframework.a)
snn: btserver.cpp $(ROOT)/posix-utils/*.hpp ../src/msp/*.hpp ../src/msp/messages.h difference_risp_train.hpp
	g++ -Wall $(INCLUDE) -I$(EDIR)/src -D_SNN -I../src -O3 -Wall btserver.cpp -o btserver

difference_risp_train.hpp: 
	$(EDIR)/compiler.py $(FDIR)/networks/$(NETWORK)

../src/msp/messages.h: ../msppg/messages.json
	cd ../msppg; make install; cd ../rpi

run: btserver
	./btserver

clean:
	rm -f btserver *.o *.hpp

edit:
	vim btserver.cpp
