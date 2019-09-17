#!/usr/bin/env python3
'''
Simple test to check SET_RANGE_AND_FLOW messages sent from Butterfly loiter-board

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

#PORT = '/dev/ttyACM0'
PORT = 'COM61'

import serial
from msppg import Parser
import sys

class RangeAndFlowParser(Parser):

    def handle_SET_RANGE_AND_FLOW(self, agl, flowx, flowy):

        print(agl, flowx, flowy)
        sys.stdout.flush()

port = serial.Serial(PORT, 115200)

parser = RangeAndFlowParser()

while True:

        c = None

        try:

            c = port.read()


        except KeyboardInterrupt:

            port.close()
            break
 
        parser.parse(c)
    

