#!/usr/bin/python3

'''
Copyright (C) 2026 Simon D. Levy

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, in version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
'''

import argparse
from argparse import ArgumentDefaultsHelpFormatter
import serial

from radiomaster import RadioMaster
from telemetry import TelemetryParser


argparser = argparse.ArgumentParser(
            formatter_class=ArgumentDefaultsHelpFormatter)

argparser.add_argument('-p', '--port', default='/dev/ttyUSB0',
                       help='Serial port for dongle')

args = argparser.parse_args()

try:
    port = serial.Serial(args.port, 115200)

except serial.SerialException:
    print('Unable to open port ' + args.port)
    exit(0)


