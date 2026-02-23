#  Copyright (C) 2026 Simon D. Levy
 
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, in version 3.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program. If not, see <http:--www.gnu.org/licenses/>.

C++ = g++ 

ROOT = ../..

SIMSENSORS = $(ROOT)/simsensors/src

HF = ..

HFSRC = $(HF)/src

HFWB = $(HF)/webots

ALL = twoexit pingpong

# Uncomment this and run with valgrind to check for memory issues
# DBGFLAGS = -g -O0

CFLAGS = -std=c++17 -Wall -Wextra $(DBGFLAGS)

DEPS = $(HFSRC)/*.hpp $(HFSRC)/pids/*.hpp  $(HFSRC)/sim/*.hpp \
	   $(HFSRC)/sim/autopilots/*.hpp $(SIMSENSORS)/parsers/webots/* \
	   $(SIMSENSORS)/sensors/* 

all: $(ALL)

trun: twoexit
	./twoexit

twoexit: twoexit.o
	$(C++) $(CFLAGS) -o twoexit twoexit.o

twoexit.o: twoexit.cpp $(DEPS)
	$(C++) $(CFLAGS) -c -I$(ROOT) -I$(HFSRC) twoexit.cpp

prun: pingpong
	./pingpong

pingpong: pingpong.o
	$(C++) $(CFLAGS) -o pingpong pingpong.o

pingpong.o: pingpong.cpp $(DEPS)
	$(C++) $(CFLAGS) -c -I$(ROOT) -I$(HFSRC) pingpong.cpp

clean:
	rm -f $(ALL) *.o *.csv
