#!/usr/bin/env python3

'''
getstate_rfcomm.py Uses MSPPG to request and handle STATE messages from flight controller IMU

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

BD_ADDR = '00:06:66:73:E3:E8'
BD_PORT = 1

from msppg import MSP_Parser as Parser, serialize_STATE_Request
import bluetooth
from time import time, sleep

parser = Parser()
request = serialize_STATE_Request()

sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((BD_ADDR, BD_PORT))

print('connected to ' + BD_ADDR)

def handler(altitude, variometer, positionX, positionY, heading, velocityForward, velocityRightward):

    print(altitude, variometer, positionX, positionY, heading, velocityForward, velocityRightward)
    sock.send(request)

parser.set_STATE_Handler(handler)

sock.send(request)

while True:

        try:

            parser.parse(sock.recv(1))

        except KeyboardInterrupt:

            sock.close()
            break

