#!/usr/bin/python3
'''
Hackflight Ground Control Station main program

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
'''

from serial import Serial
import argparse

from msp import Parser


class MyMspParser(Parser):

    def handle_STATE(self, dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi):

        print(('dx=%+3.3f dy=%+3.3f z=%+3.3f dz=%+3.3f phi=%+3.3f ' +
               'dphi=%+3.3f theta=%+3.3f dtheta=%+3.3f psi=%+3.3f ' +
               ' dpsi=%+3.3f ') %
              (dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi))


def main():

    fmtr = argparse.ArgumentDefaultsHelpFormatter

    arg_parser = argparse.ArgumentParser(formatter_class=fmtr)

    arg_parser.add_argument('-p', '--port', default='/dev/ttyUSB0')

    args = arg_parser.parse_args()

    port = Serial(args.port, 115200)

    msp_parser = MyMspParser()

    while True:

        try:

            c = port.read(1)

            msp_parser.parse(c)

        except KeyboardInterrupt:

            break


main()
