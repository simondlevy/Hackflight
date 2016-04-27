#!/usr/bin/env python

NAZE  = '/dev/ttyUSB0'
BAUDRATE = 9600
DURATION = 10

import serial
import sys
from time import time

from frsky_telemetry import FrSkyTelemetryParser

if __name__ == '__main__':

    naze = serial.Serial(NAZE, BAUDRATE)

    parser = FrSkyTelemetryParser()

    startsec = time()

    sys.stdout.write('Please arm the board ...')
    sys.stdout.flush()

    while True:

        elapsed_sec = time()-startsec

        msg = parser.parse(naze.read(1))

        if msg:
            sys.stdout.write('\n%s' % msg)

        
