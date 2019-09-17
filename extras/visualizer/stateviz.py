'''
State visualizer 

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
import math

def _errmsg(message):
    sys.stderr.write(message + '\n')
    sys.exit(1)

class _MyArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(1)

_request = msppg.serialize_STATE_Request()

class _StateParser(msppg.Parser):

    def __init__(self, readfun, writefun, closefun, visualizer):

        msppg.Parser.__init__(self)

        self.readfun = readfun
        self.writefun = writefun
        self.closefun = closefun

        # Visualizer object should provide a display() method
        self.viz = visualizer

    def handle_STATE(self, altitude, variometer, positionX, positionY, heading, velocityForward, velocityRightward):

        self.viz.display(altitude, positionX, positionY, math.degrees(heading))

        self.writefun(_request)

    def begin(self):

        self.writefun(_request)

        while True:

                try:

                    self.parse(self.readfun(1))

                except KeyboardInterrupt:

                    self.closefun()
                    break

def _open_outfile(cmdargs):

    return None if cmdargs.filename is None else open(cmdargs.filename, 'w')

def _handle_infile(visualizer, cmdargs):

    DT_SEC    = .01

    # Create a Visualizer object with trajectory
    viz = visualizer(cmdargs, 'From file: ' + cmdargs.filename)

    for line in open(cmdargs.filename):

        state = (float(s) for s in line.split())

        if not viz.display(*state):
            exit(0)

        time.sleep(DT_SEC)

def _handle_bluetooth(visualizer, cmdargs):

    try:
        import bluetooth
    except:
        _errmsg('import bluetooth failed; make sure pybluez is installed')

    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((cmdargs.bluetooth, 1))

    viz = visualizer(cmdargs, 'From bluetooth: ' + cmdargs.bluetooth, _open_outfile(cmdargs))

    parser = _StateParser(sock.recv, sock.send, sock.close, viz)

    parser.begin()

def _handle_serial(visualizer, cmdargs):

    try:
        import serial
    except:
        _errmsg('import serial failed; make sure pyserial is installed')

    port = serial.Serial(cmdargs.serial, 115200)

    viz = visualizer(cmdargs, 'From serial: ' + cmdargs.serial, _open_outfile(cmdargs))

    parser = _StateParser(port.read, port.write, port.close, viz)

    parser.begin()

def run(visualizer):

    parser = _MyArgumentParser(description='Visualize incoming vehicle-state messages.')

    parser.add_argument('-f', '--filename',    help='read state data from file')
    parser.add_argument('-b', '--bluetooth',   help='read state data from Bluetooth device')
    parser.add_argument('-s', '--serial',      help='read state data from serial port')
    parser.add_argument('-z', '--zero_angle',  help='starting angle in degrees')

    if len(sys.argv)==1:
        parser.print_help(sys.stderr)
        sys.exit(1)

    cmdargs = parser.parse_args()

    # Filename only; read it
    if cmdargs.serial is None and cmdargs.bluetooth is None and not cmdargs.filename is None:
        _handle_infile(visualizer, cmdargs)

    # Bluetooth
    if not cmdargs.bluetooth is None:
        _handle_bluetooth(visualizer, cmdargs)
    
    # Serial
    if not cmdargs.serial is None:
        _handle_serial(visualizer, cmdargs)
