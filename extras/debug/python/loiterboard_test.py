#!/usr/bin/env python3
'''
Simple test to check SET_RANGE_AND_FLOW messages sent from Butterfly loiter-board

Copyright (C) Simon D. Levy 2021

MIT License
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
    

