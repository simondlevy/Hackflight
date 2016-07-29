#!/usr/bin/env python

'''
   writeserial.py : Python test code for C++ SerialConnection class.
   Works with readserial.cpp to illustrate bytesAvailable(), readBytes() methods.

   Copyright (C) Simon D. Levy 2016

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
'''


from serial import Serial
from sys import argv

if len(argv) < 2:
    print('Usage:   %s PORTNAME' % argv[0])
    print('Example: %s /dev/ttyUSB0' % argv[0])
    exit(1)

s = Serial(argv[1])

k = 0

while True:

    s.write(chr(ord('a')+k))
    k = (k+1) % 26
