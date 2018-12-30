#!/usr/bin/env python3

'''
getstate_rfcomm.py Uses MSPPG to request and handle STATE messages from flight controller over Bluetooth

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

from msppg import Parser, serialize_STATE_Request
import bluetooth

request = serialize_STATE_Request()

class StateParser(Parser):

    def handle_STATE(altitude, variometer, positionX, positionY, heading, velocityForward, velocityRightward):
        print(altitude, variometer, positionX, positionY, heading, velocityForward, velocityRightward)
        sock.send(request)

parser = Parser()

sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((BD_ADDR, BD_PORT))

print('connected to ' + BD_ADDR)

sock.send(request)

while True:

        try:

            c = sock.recv(1)
            print('%02x' % ord(c)) 
            parser.parse(c)

        except KeyboardInterrupt:

            sock.close()
            break

