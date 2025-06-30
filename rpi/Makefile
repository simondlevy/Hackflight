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

FRO = $(ROOT)/framework-open
FROINC = -I$(FRO)/include -I$(FRO)/include/utils
FROOBJ = $(FRO)/obj/risp.o $(FRO)/obj/risp_static.o
FROLIB = -L $(FRO)/lib -lframework

INCLUDE =  -I$(ROOT) -I../src

standard: standard.cpp ../src/msp/messages.h
	g++ -Wall $(INCLUDE) -O3 -Wall -o btserver standard.cpp

simplediff: simplediff.cpp ../src/msp/messages.h
	g++ -Wall $(INCLUDE) -O3 -Wall -o btserver simplediff.cpp

snn: snn.cpp ../src/msp/messages.h
	g++ -Wall $(FROINC) $(INCLUDE) -I../src/tennlab -O3 -Wall -o btserver \
		snn.cpp  $(FROOBJ) $(FROLIB)

../src/msp/messages.h: ../msppg/messages.json
	cd ../msppg; make install; cd ../rpi

run: btserver
	./btserver $(FDIR)/networks/$(NETWORK).txt

clean:
	rm -f btserver
