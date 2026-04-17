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

import argparse
from argparse import ArgumentDefaultsHelpFormatter
import serial

try:
    from msp import Parser as MspParser
except Exception as e:
    print('%s;\nto install msp: cd ../msppg; make install' % str(e))
    exit(0)

class TelemetryParser(MspParser):

    def __init__(self):

        MspParser.__init__(self)
        self.running = True

    def handle_STATE(self, dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi):

        print(('dx=%+03.2f dy=%+03.2f z=%+03.2f dz=%+03.2f ' +
               'phi=%+5.1f dphi=%+6.1f theta=%+5.1f dtheta=%+6.1f ' +
               'psi=%+5.1f dpsi=%+5.1f') %
              (dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi))


if __name__ == '__main__':

    argparser = argparse.ArgumentParser(
            formatter_class=ArgumentDefaultsHelpFormatter)

    argparser.add_argument('-p', '--port', default='/dev/ttyUSB0',
                           help='Serial port for dongle')

    args = argparser.parse_args()

    try:
        port = serial.Serial(args.port, 115200)

    except serial.SerialException:
        print('Unable to open port ' + args.port)
        exit(0)

    telemetryParser = TelemetryParser()

    while True:

        try:

            telemetryParser.parse(port.read(1))

        except KeyboardInterrupt:

            break;

