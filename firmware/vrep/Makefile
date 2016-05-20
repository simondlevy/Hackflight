#
#   Makefile for V-REP simulator using Hackflight
#
#   Copyright (C) Simon D. Levy 2016
#
#   This file is part of Hackflight.
#
#   Hackflight is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#   Hackflight is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#   You should have received a copy of the GNU General Public License
#   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
#
 
VREP   = $(HOME)/Software/V-REP_PRO_EDU_V3_3_1_64_Linux
COMMON = $(VREP)/programming/common/

CFLAGS = -I../include -Wall -fPIC -static
CFLAGS += -D__linux
CFLAGS += -I$(HOME)/Software/V-REP_PRO_EDU_V3_3_1_64_Linux/programming/include/
EXT = so

ALL = libv_repExtHackflight.so 

all: $(ALL)

libv_repExtHackflight.so: v_repExtHackflight.cpp v_repExtHackflight.hpp
	g++ $(CFLAGS) -c v_repExtHackflight.cpp -o v_repExtHackflight.o
	g++ $(CFLAGS) -c $(COMMON)/scriptFunctionData.cpp -o scriptFunctionData.o
	g++ $(CFLAGS) -c $(COMMON)/scriptFunctionDataItem.cpp -o scriptFunctionDataItem.o
	g++ $(CFLAGS) -c $(COMMON)/v_repLib.cpp -o v_repLib.o
	g++ scriptFunctionData.o scriptFunctionDataItem.o v_repExtHackflight.o v_repLib.o \
		-o libv_repExtHackflight.$(EXT) -lpthread -ldl -shared 	

edit:
	vim v_repExtHackflight.cpp

clean:
	rm -f *.o *.so
