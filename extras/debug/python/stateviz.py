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

import msppg
import argparse
import realtime_plot
import sys
import time
import threading
import numpy as np

def errmsg(message):
    sys.stderr.write(message + '\n')
    sys.exit(1)

class MyArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(1)

request = msppg.serialize_STATE_Request()

def plotstate(altitude, variometer, positionX, positionY, heading, velocityForward, velocityRightward):

    print(altitude, variometer)

    return

class StatePlotter(realtime_plot.RealtimePlotter):

    def __init__(self):

        realtime_plot.RealtimePlotter.__init__(self, [(-1,+1), (-1,+1)], 
                phaselims=((-1,+1), (-1,+1)),
                window_name='Position',
                yticks = [(-1,0,+1),(-1,0,+1)],
                styles = ['r--', 'b-'], 
                ylabels=['X', 'Y'])
        
        self.xcurr = 0
        self.start_time = time.time()
        self.start_pos = None
        self.pos = (0,0)


    def getValues(self):

         return self.pos[0], self.pos[1], self.pos[0], self.pos[1]

class StateParser(msppg.Parser):

    def __init__(self, readfun, writefun, closefun):

        msppg.Parser.__init__(self)

        self.readfun = readfun
        self.writefun = writefun
        self.closefun = closefun

    def handle_STATE(self, altitude, variometer, positionX, positionY, heading, velocityForward, velocityRightward):
        plotstate(altitude, variometer, positionX, positionY, heading, velocityForward, velocityRightward)
        self.writefun(request)

    def begin(self):

        self.writefun(request)

        while True:

                try:

                    self.parse(self.readfun(1))

                except KeyboardInterrupt:

                    self.closefun()
                    break

def handle_file(filename, plotter):

    print('Read from file ' + filename)

    for line in open(filename):

        state = (float(s) for s in line.split())

        plotstate(*state)

        time.sleep(.1)

def handle_bluetooth(device_address, plotter):

    try:
        import bluetooth
    except:
        errmsg('import bluetooth failed; make sure pybluez is installed')

    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((device_address, 1))

    print('connected to ' + device_address)

    parser = StateParser(sock.recv, sock.send, sock.close)

    parser.begin()

def handle_serial(portname, plotter):

    try:
        import serial
    except:
        errmsg('import serial failed; make sure pyserial is installed')

    port = serial.Serial(portname, 115200)

    print('conntcted to ' + portname)

    parser = StateParser(port.read, port.write, port.close)

    parser.begin()

def handle_random(seed, plotter):

    MAG = .0005

    if seed >= 0:
        np.random.seed(seed)

    plotter.pos = np.zeros(2)

    while True:

        plotter.pos += 2*MAG * np.random.random(2) - MAG

def threadfunc(args, plotter):

    if not args.file is None:
        handle_file(args.file, plotter)

    if not args.bluetooth is None:
        handle_bluetooth(args.bluetooth, plotter)
    
    if not args.serial is None:
        handle_serial(args.serial, plotter)

    if not args.random is None:
        handle_random(args.random, plotter)

if __name__ == '__main__':

    parser = MyArgumentParser(description='Visualize incoming vehicle-state messages. Defaults to random-walk data.')

    parser.add_argument('-f', '--file',      help='read state data from file')
    parser.add_argument('-b', '--bluetooth', help='read state data from Bluetooth device')
    parser.add_argument('-s', '--serial',    help='read state data from serial port')
    parser.add_argument('-r', '--random',    nargs='?', const=-1, type=int, help='use random-walk simulation with optional non-negative random seed')

    if len(sys.argv)==1:
        parser.print_help(sys.stderr)
        sys.exit(1)

    cmdargs = parser.parse_args()

    plotter = StatePlotter()

    thread = threading.Thread(target=threadfunc, args=(cmdargs, plotter))
    thread.daemon = True
    thread.start()

    plotter.start()


