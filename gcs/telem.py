#!/usr/bin/python3
'''
Hackflight Ground Control Station program

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

import os
from sys import stdout
from serial import Serial
import argparse
from threading import Thread
from time import time, sleep

from msp import Parser


class MyMspParser(Parser):

    def __init__(self):

        Parser.__init__(self)

        self.last_received_time = 0

    def handle_STATE(self, dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi):

        print('phi=%+03.0f theta=%+03.0f psi=%+03.0f' % (phi, theta, psi))

        self.last_received_time = time()


def telemetry_threadfun(port, msp, running):

    while running[0]:

        if port.in_waiting > 0:
            msp.parse(port.read())

        elif (msp.last_received_time > 0 and
              (time() - msp.last_received_time) > 1.0):
            print('Lost connection to vehicle')
            os._exit(1)
            running[0] = False

        sleep(0)


def main():

    fmtr = argparse.ArgumentDefaultsHelpFormatter

    arg_parser = argparse.ArgumentParser(formatter_class=fmtr)

    arg_parser.add_argument('-p', '--port', default='/dev/ttyUSB0')

    args = arg_parser.parse_args()

    port = Serial(args.port, 115200)

    msp = MyMspParser()

    running = [True]

    t1 = Thread(target=telemetry_threadfun,
                args=(port, msp, running))

    t1.start()

    print('Waiting for vehicle to connect ...', end='')
    stdout.flush()

    while running[0]:

        try:

            pass

        except KeyboardInterrupt:

            break

    running[0] = False


main()
