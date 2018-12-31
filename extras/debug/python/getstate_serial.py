#!/usr/bin/env python3

'''
getstate_serial.py Uses MSPPG to request and handle STATE messages from flight controller over USB

Copyright (C) Simon D. Levy 2018

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

PORT = '/dev/ttyACM0'

from msppg import Parser, serialize_STATE_Request
import serial
from sys import stdout

request = serialize_STATE_Request()

class StateParser(Parser):

    def handle_STATE(self, altitude, variometer, positionX, positionY, heading, velocityForward, velocityRightward):
        print(altitude, variometer, positionX, positionY, heading, velocityForward, velocityRightward)
        port.write(request)

parser = StateParser()

port = serial.Serial(PORT, 115200)

port.write(request)

while True:

        try:

            parser.parse(port.read(1))

        except KeyboardInterrupt:

            port.close()
            break
