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

hackflight: board.o hackflight.o filters.o imu.o mixer.o msp.o rc.o stabilize.o
	g++ -o hackflight board.o hackflight.o filters.o imu.o mixer.o msp.o rc.o stabilize.o

board.o: board.cpp
	g++ -c -I ../firmware board.cpp

hackflight.o: ../firmware/hackflight.cpp
	g++ -c ../firmware/hackflight.cpp	

filters.o: ../firmware/filters.cpp
	g++ -c ../firmware/filters.cpp	

imu.o: ../firmware/imu.cpp
	g++ -c ../firmware/imu.cpp	

mixer.o: ../firmware/mixer.cpp
	g++ -c ../firmware/mixer.cpp	

msp.o: ../firmware/msp.cpp
	g++ -c ../firmware/msp.cpp	

rc.o: ../firmware/rc.cpp
	g++ -c ../firmware/rc.cpp	

stabilize.o: ../firmware/stabilize.cpp
	g++ -c ../firmware/stabilize.cpp	

clean:
	rm -f hackflight *.o *~

edit:
	vim board.cpp
