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
import microslam
import slamvis3d

class MyParser(msppg.MSP_Parser):

    def __init__(self, port, slam, vis):

        msppg.MSP_Parser.__init__(self)

        self.attitude = (0,0,0)
        self.altitude = 0
        self.sonars   = (0,0,0,0)

        self.port = port
        self.slam = slam
        self.vis = vis

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
        self.update()
        self.send_sonars_request()

    def attitude_handler(self, pitch, roll, heading):
        self.attitude = (pitch, roll, heading)
        self.update()
        self.send_attitude_request()
    
    def altitude_handler(self, height, vario):
        # Vario does not change from 0 on simulator, so not displayed
        self.altitude = height
        self.update()
        self.send_altitude_request()

    def update(self):

        # Update our SLAM algorithm with new telemetry, getting new pose and obstacles
        pose, obstacles = self.slam.update(self.sonars, self.attitude, self.altitude)

        print(pose)
        
        # Redraw our SLAM visualizer, exiting if user closed the window
        if not self.vis.redraw():
            exit(0)

    def send_requests(self):
        self.send_sonars_request()
        self.send_altitude_request()
        self.send_attitude_request()


if __name__ == '__main__':

    if sys.version_info.major > 2:
        print('Cannot run under Python3!')
        exit(1)

    if len(sys.argv) < 3:
        print('Usage: python3 %s PORT BAUD' % sys.argv[0])
        print('Example: python3 %s /dev/ttyUSB0 57600' % sys.argv[0])
        exit(1)

    # Connect to our telemetry provider over a serial port
    port = serial.Serial(sys.argv[1], int(sys.argv[2]), timeout=1)

    # Create a MicoSLAM algorithm object
    slam = microslam.MicroSLAM()

    # Create a 3D visualization object
    vis = slamvis3d.ThreeDSlamVis()

    # Create an MSP parser, which will use the SLAM algorithm and visualizer to 
    # analyze telemetry and display SLAM.  We also pass it the comm port, so it
    # can send new message requests.
    parser = MyParser(port, slam, vis)

    # Kick off the telemetry by sending requests to the vehicle
    parser.send_requests()

    # Loop forever, reading one byte at a time from the serial port and dispatching
    # the byte to the parser
    while True:

        # Read from serial port, exiting gracefully on CTRL-C
        try:
            c = port.read(1)
        except KeyboardInterrupt:
            break

        # Got a byte; parse it
        if len(c) == 1:             
            parser.parse(c)

        # Timed out; have parser a new set of requests.  This allows us to start and
        # stop the telemetry provider (vehicle, simulator) without having to restart
        # this program.
        else:
            parser.send_requests()  
