#!/usr/bin/env python

'''
setarmed.py : Test script for MSP arming of board

Copyright (C) Simon D. Levy, Josep, Juan 2018

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

PORT = 'COM13'          # Windows
#PORT = '/dev/ttyACM0' # Linux

DURATION = 8.0   # seconds
PERIOD   = 1.0   # switch on / off

from serial import Serial
from msppg import serialize_SET_ARMED
from time import time, sleep
from sys import stdout
import struct

if __name__ == "__main__":

    port = Serial(PORT, BAUD)

    armed = False

    start_time = time()

    prev_time = start_time

    while True:

        curr_time = time()

        if curr_time - start_time > DURATION:
            break

        if curr_time - prev_time > PERIOD:
            #port.write('\x24\x4d\x3c\x01\xd8\x01\xd8')   # Python 2 
            message = bytes([ord('$'), ord('M'), ord('<'), 0x01,0xd8,0x01,0xd8]) # Python3
            port.write(message)
            armed = not armed
            prev_time = curr_time
            break
