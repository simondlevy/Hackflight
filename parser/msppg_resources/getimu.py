#!/usr/bin/env python3

'''
getimu.py Uses MSPPG to request and handle ATTITUDE messages from flight controller IMU

Copyright (C) Rob Jones, Alec Singer, Chris Lavin, Blake Liebling, Simon D. Levy 2015

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.
This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http:#www.gnu.org/licenses/>.
'''

BAUD = 115200

from msppg import MSP_Parser as Parser, serialize_ATTITUDE_Request
import serial

from sys import argv

if len(argv) < 2:

    print('Usage: python3 %s PORT' % argv[0])
    print('Example: python3 %s /dev/ttyUSB0' % argv[0])
    exit(1)

parser = Parser()
request = serialize_ATTITUDE_Request()
port = serial.Serial(argv[1], BAUD)

def handler(pitch, roll, yaw):

    print(pitch, roll, yaw)
    port.write(request)

parser.set_ATTITUDE_Handler(handler)

port.write(request)

while True:

    try:

        parser.parse(port.read(1))

    except KeyboardInterrupt:

        break

