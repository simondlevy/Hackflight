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
import numpy as np
from threading import Thread
from sys import argv, stdout

# Change these to suit your needs
PORT = 'COM38'
BAUD = 115200

ALTITUDE_RANGE     = -0.1,1.9
VARIOMETER_RANGE   = -10,+10
NTICKS             = 10

class SerialPlotter(RealtimePlotter):

    def __init__(self):

        ranges = [rng for rng in [ALTITUDE_RANGE, VARIOMETER_RANGE]]

        RealtimePlotter.__init__(self, 
                ranges, 
                show_yvals=True,
                ylabels=['Altitude', 'Variometer'],
                yticks=[np.linspace(rng[0], rng[1], NTICKS-1) for rng in ranges],
                window_name='Altitude Estimation',
                styles=['b', 'r'])

        self.tick = 0
        self.vals = None

        self.usec_prev = 0
        self.alti_prev = 0

    def getValues(self):

         return self.vals

def _update(port, plotter):

    while True:

        dist,roll,pitch,accelx,accely,accelz,usec = [float(s) for s in port.readline().decode()[:-2].split()]

        alti = dist * np.cos(roll) * np.cos(pitch);

        if plotter.usec_prev > 0:

            print(accelz)
            stdout.flush()

            dsec = (usec-plotter.usec_prev) / 1e6

            vari = (alti - plotter.alti_prev) / dsec

            plotter.vals = alti, vari

            plotter.tick += 1

        plotter.usec_prev = usec
        plotter.alti_prev = alti

if __name__ == '__main__':

    port = argv[1] if len(argv) > 1 else PORT

    try:
        port = serial.Serial(port, BAUD)
    except serial.SerialException:
        print('Unable to open device on port %s' % PORT)
        exit(1)

    plotter = SerialPlotter()

    thread = Thread(target=_update, args = (port, plotter))
    thread.daemon = True
    thread.start()

    plotter.start()
