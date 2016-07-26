#
#   Makefile for 3DSLAM visiualization V-REP simulator plugin 
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
 
# Edit this for where you installed V-REP
VREP_DIR_LINUX = $(HOME)/Software/V-REP_PRO_EDU_V3_3_1_64_Linux
VREP_DIR_OSX   = /Applications/V-REP_PRO_EDU_V3_3_1_Mac

COMMON = $(VREP_DIR)/programming/common/
CFLAGS = -Wall -D_SIM -I$(VREP_DIR)/programming/include/ -I. -I.. -I../../firmware -Wall -fPIC -static

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

PLUGIN = libv_repExt3DSLAM.$(EXT)

libv_repExt3DSLAM.$(EXT): v_repExt3DSLAM.cpp
	g++ $(CFLAGS) -c -DVREP_DIR=\"$(VREP_LIBDIR)\" v_repExt3DSLAM.cpp 
	g++ $(CFLAGS) -c $(COMMON)/scriptFunctionData.cpp 
	g++ $(CFLAGS) -c $(COMMON)/scriptFunctionDataItem.cpp
	g++ $(CFLAGS) -c $(COMMON)/v_repLib.cpp
	g++ *.o -o libv_repExt3DSLAM.$(EXT) -lpthread -shared

install: $(PLUGIN)
	cp $(PLUGIN) $(VREP_LIBDIR)

edit:
	vim v_repExt3DSLAM.cpp

clean:
	rm -f *.o *.$(EXT) *.pyc *~


