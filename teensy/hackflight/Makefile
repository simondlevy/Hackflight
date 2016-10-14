#
#   Makefile for convenience in editing and uploading Hackflight firmware on Teensy
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

build: hackflight.ino
	arduino --verify hackflight.ino

hackflight.ino: 
	cp ../../firmware/*.* .
	mv hackflight.cpp hackflight.ino

flash:
	arduino --upload hackflight.ino

edit:
	vim board.cpp
