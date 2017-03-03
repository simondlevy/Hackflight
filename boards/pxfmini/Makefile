#
#   Makefile for Hackflight on Erle Robotics PXFMini
#
#   Copyright (C) Simon D. Levy and 
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

CFLAGS = -Wall

hackflight: main.o pxfmini.o hackflight.o filters.o imu.o mixer.o msp.o rc.o stabilize.o sonars.o hover.o
	g++ -o hackflight main.o pxfmini.o hackflight.o filters.o imu.o mixer.o msp.o rc.o stabilize.o sonars.o hover.o -lwiringPi

main.o: main.cpp
	g++ $(CFLAGS) -c -I ../firmware main.cpp

pxfmini.o: pxfmini.cpp
	g++ $(CFLAGS) -c -I ../firmware pxfmini.cpp

hackflight.o: ../firmware/hackflight.cpp
	g++ $(CFLAGS) -c ../firmware/hackflight.cpp	

filters.o: ../firmware/filters.cpp
	g++ $(CFLAGS) -c ../firmware/filters.cpp	

imu.o: ../firmware/imu.cpp
	g++ $(CFLAGS) -c ../firmware/imu.cpp	

mixer.o: ../firmware/mixer.cpp
	g++ $(CFLAGS) -c ../firmware/mixer.cpp	

msp.o: ../firmware/msp.cpp
	g++ $(CFLAGS) -c ../firmware/msp.cpp	

rc.o: ../firmware/rc.cpp
	g++ $(CFLAGS) -c ../firmware/rc.cpp	

stabilize.o: ../firmware/stabilize.cpp
	g++ $(CFLAGS) -c -I. ../firmware/stabilize.cpp	

sonars.o: ../firmware/sonars.cpp
	g++ $(CFLAGS) -c -I. ../firmware/sonars.cpp	

hover.o: ../firmware/hover.cpp
	g++ $(CFLAGS) -c -I. ../firmware/hover.cpp	

clean:
	rm -f hackflight *.o *~

edit:
	vim pxfmini.cpp

run:
	./hackflight
