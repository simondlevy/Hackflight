#!/usr/bin/env python3

'''
getimu.py Uses MSPPG to request and handle ATTITUDE_RADIANS messages from flight controller IMU

Copyright (C) Rob Jones, Alec Singer, Chris Lavin, Blake Liebling, Simon D. Levy 2015

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify
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

#PORT = 'COM13'          # Windows
PORT = '/dev/ttyACM0' # Linux

from msppg import Parser as Parser, serialize_ATTITUDE_RADIANS_Request
import serial

request = serialize_ATTITUDE_RADIANS_Request()

class ImuParser(Parser):

    def hande_ATTITUDE_RADIANS(sef, pitch, roll, yaw):
        print(pitch, roll, yaw)
        port.write(request)

parser = ImuParser()

port = serial.Serial(PORT, BAUD)

port.write(request)

while True:

    try:

        parser.parse(port.read(1))

    except KeyboardInterrupt:

        break

