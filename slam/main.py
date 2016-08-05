#!/usr/bin/env python2

'''
main.py : Runs and displays 3D SLAM using sensor telemetry retrieved over comm port.

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

import serial
import sys
import msppg
import slamvis3d
import microslam

if sys.version_info.major > 2:
    print('Cannot run under Python3!')
    exit(1)

if len(sys.argv) < 3:
    print('Usage: python3 %s PORT BAUD' % sys.argv[0])
    print('Example: python3 %s /dev/ttyUSB0 57600' % sys.argv[0])
    exit(1)

class MyParser(msppg.MSP_Parser):

    def __init__(self, port):

        msppg.MSP_Parser.__init__(self)

        self.attitude = (0,0,0)
        self.altitude = 0
        self.sonars   = (0,0,0,0)

        self.port = port
        self.count = 0

        self.sonars_request = msppg.serialize_SONARS_Request()
        self.attitude_request = msppg.serialize_ATTITUDE_Request()
        self.altitude_request = msppg.serialize_ALTITUDE_Request()

        self.set_SONARS_Handler(self.sonars_handler)
        self.set_ATTITUDE_Handler(self.attitude_handler)
        self.set_ALTITUDE_Handler(self.altitude_handler)

    def send_sonars_request(self):
        self.port.write(self.sonars_request)

    def send_altitude_request(self):
        self.port.write(self.altitude_request)

    def send_attitude_request(self):
        self.port.write(self.attitude_request)

    def sonars_handler(self, back, front, left, right):
        self.sonars = (back, front, left, right)
        self.report()
        self.send_sonars_request()

    def attitude_handler(self, pitch, roll, heading):
        self.attitude = (pitch, roll, heading)
        self.report()
        self.send_attitude_request()
    
    def altitude_handler(self, height, vario):
        # Vario does not change from 0 on simulator, so not displayed
        self.altitude = height
        self.report()
        self.send_altitude_request()

    def report(self):

        print('%4d ------------------------' % self.count)

        print('Sonars: back: %d cm  front: %d cm left: %d cm right: %d cm' % 
                (self.sonars[0], self.sonars[1], self.sonars[2], self.sonars[3]))

        print('Altitude: %d cm' % (self.altitude))

        print ('Attitude: Pitch: %+3.1f deg   Roll: %+3.1f deg  Heading: %+3.1f deg' % 
                (self.attitude[0]/10., self.attitude[1]/10., self.attitude[2]/10.))

        # Make sure we can see progress when vehicle is stationary
        self.count += 1

    def send_requests(self):
        self.send_sonars_request()
        self.send_altitude_request()
        self.send_attitude_request()


port = serial.Serial(sys.argv[1], int(sys.argv[2]), timeout=1)

parser = MyParser(port)

parser.send_requests()

while True:

    # Read from serial port, exiting gracefully on CTRL-C
    try:
        c = port.read(1)
    except KeyboardInterrupt:
        break

    # Got a byte; parse it
    if len(c) == 1:             
        parser.parse(c)

    # Timed out; have parser a new set of requests
    else:
        parser.send_requests()  
