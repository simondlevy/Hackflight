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

ROOT = ../..

SIMSENSORS = $(ROOT)/simsensors/src

HF = ..

HFSRC = $(HF)/src

HFWB = $(HF)/webots

ALL = twoexit pingpong 

CFLAGS = -O3 -std=c++17 -Wall -Wextra # -fsanitize=address

DEPS = $(HFSRC)/*.hpp $(HFSRC)/pids/*.hpp  $(HFSRC)/simulator/*.hpp \
	   $(HFSRC)/autopilots/*.hpp $(SIMSENSORS)/parsers/webots/* \
	   $(SIMSENSORS)/sensors/* 

CC = g++ $(CFLAGS) -c -I$(ROOT) -I$(HFSRC)

all: $(ALL)

trun: twoexit
	./twoexit

prun: pingpong
	./pingpong

twoexit: twoexit.o
	g++ -o twoexit twoexit.o

pingpong: pingpong.o
	g++ -o pingpong pingpong.o

twoexit.o: twoexit.cpp $(DEPS)
	$(CC) twoexit.cpp

pingpong.o: pingpong.cpp $(DEPS)
	$(CC) pingpong.cpp

clean:
	rm -f $(ALL) *.o *.csv
