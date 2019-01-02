#!/usr/bin/env python3
'''
stateviz.py: State visualizer 

Dependencies: numpy, matplotlib, https://github.com/simondlevy/RealtimePlotter

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

import argparse
import sys

def errmsg(message):
    sys.stderr.write(message + '\n')
    sys.exit(1)

class MyArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(1)

def handle_file(filename):

    print('Read from file ' + filename)

def handle_bluetooth(device_address):

    try:
        import bluetooth
    except:
        errmsg('import bluetooth failed; make sure pybluez is installed')

    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((device_address, 1))

    print('connected to ' + device_address)

def handle_serial(portname):

    try:
        import serial
    except:
        errmsg('import serial failed; make sure pyserial is installed')

    print('Read from serial port ' + portname)

if __name__ == '__main__':

    parser = MyArgumentParser(description='Visualize incoming vehicle-state messages.')

    parser.add_argument('-f', '--file',      help='read state data from file')
    parser.add_argument('-b', '--bluetooth', help='read state data from Bluetooth device')
    parser.add_argument('-s', '--serial',    help='read state data from serial port')

    if len(sys.argv)==1:
        parser.print_help(sys.stderr)
        sys.exit(1)

    args = parser.parse_args()

    if not args.file is None:
        handle_file(args.file)

    if not args.bluetooth is None:
        handle_bluetooth(args.bluetooth)
    
    if not args.serial is None:
        handle_serial(args.serial)

