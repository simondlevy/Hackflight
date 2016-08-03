#!/usr/bin/env python2

'''
slammin.py : Runs SLAM from sensor telemetry retrieved over comm port

Copyright (C) Matt Lubas, Alfredo Rwagaju, and Simon D. Levy 2016

This code is free software: you can redistribute it and/or modify
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

from msppg import MSP_Parser as Parser, serialize_SONARS_Request, serialize_ATTITUDE_Request
import serial
from sys import argv, version_info

if version_info.major > 2:
    print('Cannot run under Python3!')
    exit(1)

if len(argv) < 3:

    print('Usage: python3 %s PORT BAUD' % argv[0])
    print('Example: python3 %s /dev/ttyUSB0 57600' % argv[0])
    exit(1)

class MyParser(Parser):

    def __init__(self, port):

        Parser.__init__(self)

        self.port = port
        self.count = 0
        self.sonars_request = serialize_SONARS_Request()
        self.attitude_request = serialize_ATTITUDE_Request()
        self.set_SONARS_Handler(self.sonars_handler)

    def send_requests(self):
        self.port.write(self.sonars_request)

    def sonars_handler(self, back, front, left, right):
        print('%4d: Sonars: back: %d   front: %d  left: %d  right: %d' % 
                (self.count, back, front, left, right))
        self.count += 1
        self.send_requests()

port = serial.Serial(argv[1], int(argv[2]), timeout=1)

parser = MyParser(port)

parser.send_requests()

while True:

    c = port.read(1)

    if len(c) == 1:             # got a byte; parse it
        parser.parse(c)
    else:
        parser.send_requests()  # timed out; have parser a new set of requests
