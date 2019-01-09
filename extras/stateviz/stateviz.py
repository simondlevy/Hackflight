#!/usr/bin/env python3
'''
stateviz.py: State visualizer 

Dependencies: numpy, matplotlib, pyserial, pybluez, https://github.com/simondlevy/PyRoboViz

Copyright (C) 2018 Simon D. Levy

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

import msppg
import argparse
import sys
import time
from math import degrees
from roboviz import Visualizer

MAP_SIZE_PIXELS = 800
MAP_SIZE_METERS = 32

def errmsg(message):
    sys.stderr.write(message + '\n')
    sys.exit(1)

class MyArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(1)

request = msppg.serialize_STATE_Request()

class StateParser(msppg.Parser):

    def __init__(self, readfun, writefun, closefun, label):

        msppg.Parser.__init__(self)

        self.readfun = readfun
        self.writefun = writefun
        self.closefun = closefun

        # Create a Visualizer object with trajectory
        self.viz = Visualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, label, True)

    def handle_STATE(self, altitude, variometer, positionX, positionY, heading, velocityForward, velocityRightward):

        self.viz.setPose(0, 0, degrees(heading))

        self.viz.refresh()

        self.writefun(request)

    def begin(self):

        self.writefun(request)

        while True:

                try:

                    self.parse(self.readfun(1))

                except KeyboardInterrupt:

                    self.closefun()
                    break

def handle_file(filename):

    DT_SEC    = .01

    print('Read from file ' + filename)

    # Create a Visualizer object with trajectory
    viz = Visualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'From file: ' + filename, True)

    for line in open(filename):

        state = (float(s) for s in line.split())

        viz.setPose(*state)

        viz.refresh()

        time.sleep(DT_SEC)

def handle_bluetooth(device_address):

    try:
        import bluetooth
    except:
        errmsg('import bluetooth failed; make sure pybluez is installed')

    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((device_address, 1))

    print('connected to ' + device_address)

    parser = StateParser(sock.recv, sock.send, sock.close, 'From Bluetooth: ' + device_address)

    parser.begin()

def handle_serial(portname):

    try:
        import serial
    except:
        errmsg('import serial failed; make sure pyserial is installed')

    port = serial.Serial(portname, 115200)

    print('conntcted to ' + portname)

    parser = StateParser(port.read, port.write, port.close, 'From serial: ' + portname)

    parser.begin()

def threadfunc(args):

    if not args.file is None:
        handle_file(args.file)

    if not args.bluetooth is None:
        handle_bluetooth(args.bluetooth)
    
    if not args.serial is None:
        handle_serial(args.serial)

if __name__ == '__main__':

    parser = MyArgumentParser(description='Visualize incoming vehicle-state messages.')

    parser.add_argument('-f', '--file',      help='read state data from file')
    parser.add_argument('-b', '--bluetooth', help='read state data from Bluetooth device')
    parser.add_argument('-s', '--serial',    help='read state data from serial port')

    if len(sys.argv)==1:
        parser.print_help(sys.stderr)
        sys.exit(1)

    cmdargs = parser.parse_args()

    if not cmdargs.file is None:
        handle_file(cmdargs.file)

    if not cmdargs.bluetooth is None:
        handle_bluetooth(cmdargs.bluetooth)
    
    if not cmdargs.serial is None:
        handle_serial(cmdargs.serial)
