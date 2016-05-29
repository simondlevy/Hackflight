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

CFLAGS = -Wall -I. -I../include -I../firmware -Wall -fPIC -static
CFLAGS += -D__linux
CFLAGS += -I$(HOME)/Software/V-REP_PRO_EDU_V3_3_1_64_Linux/programming/include/
EXT = so

ALL = libv_repExtHackflight.so 

all: $(ALL)

libv_repExtHackflight.so: *.cpp *.hpp ../firmware/*.cpp ../firmware/*.hpp
	g++ $(CFLAGS) -c v_repExtHackflight.cpp 
	g++ $(CFLAGS) -c $(COMMON)/scriptFunctionData.cpp 
	g++ $(CFLAGS) -c $(COMMON)/scriptFunctionDataItem.cpp
	g++ $(CFLAGS) -c $(COMMON)/v_repLib.cpp
	g++ $(CFLAGS) -c ../firmware/imu.cpp
	g++ $(CFLAGS) -c ../firmware/mixer.cpp
	g++ $(CFLAGS) -c ../firmware/msp.cpp
	g++ $(CFLAGS) -c ../firmware/mw.cpp
	g++ $(CFLAGS) -c ../firmware/pid.cpp
	g++ $(CFLAGS) -c ../firmware/rc.cpp
	g++ *.o -o libv_repExtHackflight.$(EXT) -lpthread -ldl -shared 	

edit:
	vim v_repExtHackflight.cpp

commit:
	git commit -a --allow-empty-message -m ""
	git push

clean:
	rm -f *.o *.so *.pyc *~
