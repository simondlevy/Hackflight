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

PORT = 'COM60'          # Windows
#PORT = '/dev/ttyACM0' # Linux

from msppg import MSP_Parser as Parser, serialize_STATE_Request
import serial

def handler(altitude, variometer, positionX, positionY, heading, velocityForward, velocityRightward):

    print(altitude, variometer, positionX, positionY, heading, velocityForward, velocityRightward)
    port.write(request)

if __name__ == '__main__':

    parser = Parser()
    request = serialize_STATE_Request()
    port = serial.Serial(PORT, BAUD)

    parser.set_STATE_Handler(handler)

    port.write(request)

    while True:

        try:

            parser.parse(port.read(1))

        except KeyboardInterrupt:

            break

