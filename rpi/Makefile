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

FRO = $(HOME)/framework-open
FROINC = -I$(FRO)/include -I$(FRO)/include/utils
FROSRC = $(FRO)/src

CXX_SOURCES += $(FRO)/src/risp_static.cpp
CXX_SOURCES += $(FRO)/src/framework.cpp
CXX_SOURCES += $(FRO)/src/processor_help.cpp
CXX_SOURCES += $(FRO)/src/properties.cpp

ROOT = $(HOME)

INCLUDE = -I$(ROOT) -I../src

standard: standard.o 
	g++ -O3 -o btserver standard.o 

standard.o: standard.cpp ../src/msp/messages.h
	g++ -Wall $(INCLUDE) -O3 -Wall -c standard.cpp

snn: snn.o risp.o risp_static.o framework.o processor_help.o properties.o
	g++ -O3 -o btserver snn.o risp.o risp_static.o framework.o processor_help.o properties.o

snn.o: snn.cpp ../src/msp/messages.h
	g++ -Wall $(INCLUDE) -O3 -Wall -c snn.cpp

risp.o: 
	g++ -O3 -Wall $(FROINC) $(FROSRC)/risp.cpp

risp_static.o: 
	g++ -O3 -Wall $(FROINC) $(FROSRC)/risp_static.cpp

framework.o: 
	g++ -O3 -Wall $(FROINC) $(FROSRC)/framework.cpp

processor_help.o: 
	g++ -O3 -Wall $(FROINC) $(FROSRC)/processor_help.cpp

properties.o: 
	g++ -O3 -Wall $(FROINC) $(FROSRC)/properties.cpp

../src/msp/messages.h: ../msppg/messages.json
	cd ../msppg; make install; cd ../rpi

run: btserver
	./btserver $(FDIR)/networks/$(NETWORK).txt

clean:
	rm -f btserver *.o $(NETWORK).hpp
