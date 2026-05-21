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

import numpy as np
import argparse
from argparse import ArgumentDefaultsHelpFormatter
import serial
import time

RealtimePlotter = None
try:
    from realtime_plot import RealtimePlotter
except Exception:
    pass

try:
    from __msp__ import Parser as MspParser
except Exception as e:
    print('%s;\nto install msp: cd ../msppg; make install' % str(e))
    exit(0)


class Telemetry(MspParser):

    PLOTTER_Z_RANGE = 0, 2
    PLOTTER_DZ_RANGE = -1, +1
    PLOTTER_DATA_SIZE = 100

    def __init__(self):

        MspParser.__init__(self)

        argparser = argparse.ArgumentParser(
                formatter_class=ArgumentDefaultsHelpFormatter)

        argparser.add_argument('-o', '--outfile', help='CSV file for logging')

        argparser.add_argument('-r', '--realtime', action='store_true',
                               help='Real-time plot')

        argparser.add_argument('-p', '--port', default='/dev/ttyUSB0',
                               help='Serial port for dongle')

        args = argparser.parse_args()

        try:
            self.port = serial.Serial(args.port, 115200)

        except serial.SerialException:
            print('Unable to open port ' + args.port)
            exit(1)

        self.outfile = None

        if args.outfile is not None:

            try:

                self.outfile = open(args.outfile, 'w')

                self.outfile.write(
                        'dx,dy,z,dz,phi,dphi,theta,dtheta,psi,dpsi\n')

            except Exception as e:
                print('Unable to open log file %s: %s' %
                      (args.outfile, str(e)))
                exit(1)

        self.plotter = None

        self.plotter_data = (
                np.zeros(self.PLOTTER_DATA_SIZE),
                np.zeros(self.PLOTTER_DATA_SIZE))

        if args.realtime:

            if RealtimePlotter is None:
                print('Realtime plotter not installed')
                exit(1)

            else:
                self.plotter = RealtimePlotter(
                        self,
                        (self.PLOTTER_Z_RANGE, self.PLOTTER_DZ_RANGE),
                        size=self.PLOTTER_DATA_SIZE,
                        show_yvals=True,
                        window_name='Flight Telemetry',
                        yticks=(
                            self.PLOTTER_Z_RANGE,
                            (self.PLOTTER_DZ_RANGE[0],
                             0, self.PLOTTER_DZ_RANGE[1])),
                        ylabels=('Z (m)', 'dZ/dt (m/s)'),
                        styles=('b-', 'g-'))

        self.running = True

    def step(self):

        try:
            self.parse(self.port.read(1))

        except serial.SerialException:
            print('Unable to read telemtry from port')

    def read(self):
        '''For RealtimePlotter'''
        self.step()
        return self.plotter_data

    def handle_STATE(self, dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi):

        if self.outfile is None and self.plotter is None:

            print(('dx=%+03.2f dy=%+03.2f z=%+03.2f dz=%+03.2f ' +
                   'phi=%+5.1f dphi=%+6.1f theta=%+5.1f dtheta=%+6.1f ' +
                   'psi=%+5.1f dpsi=%+5.1f') %
                  (dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi))

        elif self.outfile is not None:

            self.outfile.write('%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n' %
                               (time.time(), dx, dy, z, dz,
                                phi, dphi, theta, dtheta, psi, dpsi))

        self.plotter_data = self._roll_data(0, z), self._roll_data(1, dz)

    def _roll_data(self, index, newval):

        data = np.roll(self.plotter_data[index], -1)
        data[-1] = newval
        return data


if __name__ == '__main__':

    telemetry = Telemetry()

    if telemetry.plotter is not None:

        telemetry.plotter.start()

    else:

        while True:

            try:
                telemetry.step()

            except KeyboardInterrupt:
                break
