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

ROOT = $(HOME)/Desktop

FDIR = $(HOME)/Desktop/framework

PDIR =  $(FDIR)/processors/risp

CXX_SOURCES += $(FDIR)/src/io_stream.cpp
CXX_SOURCES += $(FDIR)/src/jspace.cpp
CXX_SOURCES += $(FDIR)/src/network.cpp
CXX_SOURCES += $(FDIR)/src/properties.cpp
CXX_SOURCES += $(FDIR)/src/spike_encoding.cpp
CXX_SOURCES += $(PDIR)/src/processor.cpp
CXX_SOURCES += $(PDIR)/src/static_proc.cpp

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
	g++ $(INCLUDE) -D_SNN -I../src -O3 -Wall btserver.cpp -o btserver

../src/msp/messages.h: ../msppg/messages.json
	cd ../msppg; make install; cd ../rpi

run: btserver
	./btserver

clean:
	rm -f btserver *.o

edit:
	vim btserver.cpp
