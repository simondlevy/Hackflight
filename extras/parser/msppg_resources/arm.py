#!/usr/bin/env python

'''
arm.py : Test script for MSP arming of board

Copyright (C) Simon D. Levy, Pep Marti, Juan Gallostra Acin 2018

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

from serial import Serial
from msppg import serialize_SET_ARMED

if __name__ == "__main__":

    port = Serial(PORT, BAUD)

    port.write(serialize_SET_ARMED(True))

