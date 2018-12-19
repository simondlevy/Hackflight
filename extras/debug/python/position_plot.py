#!/usr/bin/env python3
'''
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

import serial
from realtime_plot import RealtimePlotter
from threading import Thread
from time import time

# Change these to suit your needs
PORT = '/dev/ttyACM0'
BAUD = 115200
RANGE = (-1,+1)
DELAY = 5

class SerialPlotter(RealtimePlotter):

    def __init__(self):

        RealtimePlotter.__init__(self, [RANGE], 
                window_name='Serial input',
                yticks = [RANGE],
                styles = ['b-'])

        self.xcurr = 0
        self.ycurr = 0
        
        self.start_time = time()
        self.start_pos = None


    def getValues(self):

         return (self.ycurr,)

def _update(port, plotter):

    msg = ''

    while True:

        c = port.read().decode()

        if c == '\n':
            
            try:

                pos = tuple((float(v) for v in msg.split(',')))
                
            except:
                
                pass

            if plotter.start_pos is None:

                if (time() - plotter.start_time) > DELAY:

                    plotter.start_pos = pos
            else:

                plotter.ycurr = pos[0] - plotter.start_pos[0]
            
                print(plotter.ycurr)
            
            msg = ''
            
        else:
            
            msg += c

        plotter.xcurr += 1

if __name__ == '__main__':

    try:
        port = serial.Serial(PORT, BAUD)
    except serial.SerialException:
        print('Unable to access device on port %s' % PORT)
        exit(1)

    plotter = SerialPlotter()

    thread = Thread(target=_update, args = (port, plotter))
    thread.daemon = True
    thread.start()

    plotter.start()
