#
#   Makefile for HelloHackflight demo
#
#   Copyright (C) Simon D. Levy 2017
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
 
CFLAGS = -std=c++11 -Wall -D_SIM 

TARGET = HelloHackflight

FIRMDIR = ../firmware

all: $(TARGET)

$(TARGET): main.cpp $(FIRMDIR)/*.cpp $(FIRMDIR)/*.h
	g++ $(CFLAGS) -c -I$(FIRMDIR) main.cpp
	g++ $(CFLAGS) -c -I$(FIRMDIR) board.cpp
	g++ $(CFLAGS) -c $(FIRMDIR)/imu.cpp
	g++ $(CFLAGS) -c $(FIRMDIR)/mixer.cpp
	g++ $(CFLAGS) -c $(FIRMDIR)/msp.cpp
	g++ $(CFLAGS) -c $(FIRMDIR)/hackflight.cpp
	g++ $(CFLAGS) -c -I. $(FIRMDIR)/stabilize.cpp
	g++ $(CFLAGS) -c $(FIRMDIR)/filters.cpp
	g++ $(CFLAGS) -c $(FIRMDIR)/rc.cpp
	g++ $(CFLAGS) -c $(FIRMDIR)/filters.cpp
	g++ *.o -o $(TARGET)

clean:
	rm -f $(TARGET) *.o *~

