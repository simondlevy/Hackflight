#!/usr/bin/env python3

'''
test_rfcomm.py Simple test to send characters to flight controller over Bluetooth

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

import bluetooth
from time import sleep

sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((BD_ADDR, BD_PORT))

print('connected to ' + BD_ADDR)

k = 0

while True:

        try:

            sock.send(chr(k+97).encode())

            k = (k+1) % 26

            sleep(.01)

        except KeyboardInterrupt:

            sock.close()
            break

