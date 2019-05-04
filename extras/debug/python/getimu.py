#!/usr/bin/env python3

'''
getimu.py Uses MSPPG to request and handle IMU_SI messages from flight controller

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

#PORT = 'COM13'          # Windows
PORT = '/dev/ttyACM0' # Linux

from msppg import Parser, serialize_IMU_SI_Request
import serial

class ImuParser(Parser):

    def handle_IMU_SI(self, ax, ay, az, gx, gy, gz):

        print('ax: %+3.3f  ay: %+3.3f  az: %+3.3f | gx: %+3.3f  gy: %+3.3f  gz: %+3.3f ' % (ax, ay, az, gx, gy, gz))
        port.write(request)

if __name__ == '__main__':

    parser = ImuParser()
    request = serialize_IMU_SI_Request()
    port = serial.Serial(PORT, BAUD)

    port.write(request)

    while True:

        try:

            parser.parse(port.read(1))

        except KeyboardInterrupt:

            break

