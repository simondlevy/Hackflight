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

        argparser.add_argument('-o', '--outfile', help='CSV file for logging',
                               default='log.csv')

        argparser.add_argument('-r', '--realtime', action='store_true',
                               help='Real-time plot')

        argparser.add_argument('-p', '--port', default='/dev/tinypico',
                               help='Serial port for dongle')

        argparser.add_argument('-c', '--continuous_logging',
                               action='store_true',
                               help='Continuous logging')

        args = argparser.parse_args()

        try:
            self.port = serial.Serial(args.port, 115200)

        except serial.SerialException:
            print('Unable to open port ' + args.port)
            exit(1)

        self.start_time = time.time()

        self.outfile = None

        self.continuous_logging = args.continuous_logging

        try:

            self.outfile = open(args.outfile, 'w')

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

    def handle_TELEMETRY(self, mode, volts, thrust, roll, pitch, yaw, dx, dy,
                         z, dz, phi, dphi, theta, dtheta, psi, dpsi):

        modes = 'idle', 'armed', 'hover', 'autopilot'

        print('mode=%s' % modes[int(mode)], end=' | ')
        print('volts=%3.1f' % volts, end=' | ')
        print('thr=%+3.3f' % thrust, end=' | ')
        print('rol=%+3.3f' % roll, end=' | ')
        print('pit=%+3.3f' % pitch, end=' | ')
        print('yaw=%+3.3f' % yaw, end=' | ')
        print('phi=%+5.1f' % phi, end=' | ')
        print('theta=%+5.1f' % theta, end=' | ')
        print('psi=%+5.1f' % psi, end=' | ')
        print('dpsi=%+5.1f' % dpsi, end=' | ')
        print('dx=%+3.3f' % dx, end=' | ')
        print('dy=%+3.3f' % dy, end=' | ')
        print('z=%3.3f' % z, end=' | ')
        print('dz=%+3.3f' % dz)

        self._log_when_armed(
                mode,
                (mode, volts, thrust, roll, pitch, yaw, dx, dy, z, dz, phi,
                 dphi, theta, dtheta, psi, dpsi))

        self.plotter_data = self._roll_data(0, z), self._roll_data(1, dz)

    def handle_PID(
            self, mode, target, actual, error, integral, derivative, output):

        print('target=%+6.1f' % target, end=' | ')
        print('actual=%+6.1f' % actual, end=' | ')
        print('error=%+6.1f' % error, end=' | ')
        print('integral=%+05.1f' % integral, end=' | ')
        print('derivative=%+08.1f' % derivative, end=' | ')
        print('output=%+f' % output)

        self._log_when_armed(
                mode,
                (target, actual, error, integral, derivative, output))

    def _roll_data(self, index, newval):

        data = np.roll(self.plotter_data[index], -1)
        data[-1] = newval
        return data

    def _log_when_armed(self, mode, data):

        if self.continuous_logging or mode > 0 or True:
            t = time.time() - self.start_time
            fmt = ('%f,' % t) + ','.join(['%f' for _ in data]) + '\n'
            self.outfile.write(fmt % data)


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
