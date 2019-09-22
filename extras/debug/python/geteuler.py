#!/usr/bin/env python3
'''
Uses MSPPG to request and handle attitude messages from flight controller

Copyright (C) Simon D. Levy 2019

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

PORT = 'COM9'          # Windows
#PORT = '/dev/ttyACM0' # Linux

from msppg import Parser, serialize_ATTITUDE_RADIANS_Request
import serial
from time import sleep
from sys import stdout

class AttitudeParser(Parser):

    def handle_ATTITUDE_RADIANS(self, roll, pitch, yaw):

        print('%+3.3f %+3.3f %+3.3f' % (roll, pitch, yaw))
        stdout.flush()
        port.write(request)

if __name__ == '__main__':

    parser = AttitudeParser()
    request = serialize_ATTITUDE_RADIANS_Request()
    port = serial.Serial(PORT, BAUD)

    # Connecting causes reboot on ESP32
    sleep(1)

    port.write(request)

    while True:

        try:

            parser.parse(port.read(1))

        except KeyboardInterrupt:

            break

