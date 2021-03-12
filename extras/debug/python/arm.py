#!/usr/bin/env python3
'''
Test script for MSP arming of board

Copyright (C) Simon D. Levy, Pep Marti, Juan Gallostra Acin 2021

MIT License
'''

BAUD = 115200

#PORT = 'COM13'          # Windows
PORT = '/dev/ttyACM0' # Linux

from serial import Serial
from msppg import serialize_SET_ARMED

if __name__ == "__main__":

    port = Serial(PORT, BAUD)

    port.write(serialize_SET_ARMED(True))

