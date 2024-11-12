#
# Makefile for C++ / Haskell-bbased MultiSim flight controller
#
# Copyright (C) 2022 Simon D. Levy
# 
#  This file is part of SimFlightControl.
#
#  SimFlightControl is free software: you can redistribute it and/or modify it
#  under the terms of the GNU General Public License as published by the Free
#  Software Foundation, either version 3 of the License, or (at your option)
#  any later version.
#
#  SimFlightControl is distributed in the hope that it will be useful, but
#  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
#  or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
#  more details.
#
#  You should have received a copy of the GNU General Public License along with
#  SimFlightControl. If not, see <https://www.gnu.org/licenses/>.


HF_SRC = hfsrc/

ALL = hackflight

all: $(ALL)

CFLAGS = -Wall -std=c++11

all: $(ALL)

hackflight: hackflight.o 
	g++ -o hackflight hackflight.o 

hackflight.o: hackflight.cpp \
	$(HF_SRC)/*.h \
	$(HF_SRC)/*/*.h \
	$(HF_SRC)/mixers/*.h \
	$(HF_SRC)/mixers/fixedpitch/*.h
	g++ $(CFLAGS) -I$(HF_SRC) -Isockets -c hackflight.cpp

run: hackflight
	./hackflight

edit:
	vim hackflight.cpp

clean:
	rm -rf $(ALL) *.o *~
