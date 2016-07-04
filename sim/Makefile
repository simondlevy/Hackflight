#
#   Makefile for V-REP simulator using Hackflight
#
#   Copyright (C) Simon D. Levy and Matt Lubas 2016
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
 
# Edit this for where you installed V-REP
VREP_DIR_LINUX = $(HOME)/Software/V-REP_PRO_EDU_V3_3_1_64_Linux
VREP_DIR_OSX   = /Applications/V-REP_PRO_EDU_V3_3_1_Mac

# Un-comment if you want to build for the companion board:
# Linux only, requires OpenCV
#COMPANION=COMPANION

COMMON = $(VREP_DIR)/programming/common/
CFLAGS = -Wall -D_SIM -I$(VREP_DIR)/programming/include/ -I. -I../firmware -Wall -fPIC -static

OS = $(shell uname -s)
ifeq ($(OS), Linux)
	EXT = so
	CFLAGS += -I/usr/include/SDL
	VREP_DIR = $(VREP_DIR_LINUX)
    VREP_LIBDIR = $(VREP_DIR)
else
	EXT = dylib
	CFLAGS += -I/Library/Frameworks/SDL2.framework/Headers/
	JOYLIB = -lSDL2
	VREP_DIR = $(VREP_DIR_OSX)
    VREP_LIBDIR = $(VREP_DIR)/vrep.app/Contents/MacOS/
endif

PLUGIN = libv_repExtHackflight.$(EXT)

all: $(PLUGIN) 

libv_repExtHackflight.$(EXT): *.cpp *.hpp ../firmware/*.cpp ../firmware/*.hpp Makefile
	g++ $(CFLAGS) -c v_repExtHackflight.cpp 
	g++ $(CFLAGS) -c -D_$(COMPANION) -DVREP_DIR=\"$(VREP_LIBDIR)\" companion.cpp 
	g++ $(CFLAGS) -c $(COMMON)/scriptFunctionData.cpp 
	g++ $(CFLAGS) -c $(COMMON)/scriptFunctionDataItem.cpp
	g++ $(CFLAGS) -c $(COMMON)/v_repLib.cpp
	g++ $(CFLAGS) -c ../firmware/imu.cpp
	g++ $(CFLAGS) -c ../firmware/mixer.cpp
	g++ $(CFLAGS) -c ../firmware/msp.cpp
	g++ $(CFLAGS) -c ../firmware/mw.cpp
	g++ $(CFLAGS) -c ../firmware/pid.cpp
	g++ $(CFLAGS) -c ../firmware/rc.cpp
	g++ $(CFLAGS) -c ../firmware/baro.cpp
	g++ $(CFLAGS) -c ../firmware/position.cpp
	g++ $(CFLAGS) -c ../firmware/hover.cpp
	g++ $(CFLAGS) -c ../firmware/filters.cpp
	g++ *.o -o libv_repExtHackflight.$(EXT) -lpthread -shared -lopencv_core -lopencv_highgui $(JOYLIB)

install: $(PLUGIN) hackflight_companion.py
	cp $(PLUGIN) $(VREP_LIBDIR)
	cp hackflight_companion.py $(VREP_LIBDIR)
	cp socket_server.py $(VREP_LIBDIR)

uninstall:
	rm -f $(VREP_DIR)/$(PLUGIN)

edit:
	vim v_repExtHackflight.cpp

flakes:
	pyflakes hackflight_companion.py

release: $(PLUGIN)
	cp $(PLUGIN) Release

commit:
	git commit -a --allow-empty-message -m ""
	git push

clean:
	rm -f *.o *.$(EXT) *.pyc *~

