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
from inputs import get_gamepad
from threading import Thread
from time import time, sleep

from msp import Parser

class MyMspParser(Parser):

    def __init__(self, gamepad_vals):

        Parser.__init__(self)

        self.gamepad_vals = gamepad_vals

    def handle_STATE(self, dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi):

        c = self.gamepad_vals

        print('phi=%+3.3f theta=%+3.3f psi=%+3.3f | c1=%d c2=%d c3=%d c4=%d c5=%d c6=%d' %
              (phi, theta, psi, c[0], c[1], c[2], c[3], c[4], c[5]))

def gamepad_threadfun(gamepad_vals, running):

    AXIS_MAP = {'X': 0, 'Y': 1, 'Z': 2, 'RX': 3, 'RY': 4, 'RZ':5}

    while running[0]:

        for event in get_gamepad():

            code = str(event.code)

            if 'ABS' in code:

                axis = AXIS_MAP[code[4:]]

                gamepad_vals[axis] = event.state


def main():

    fmtr = argparse.ArgumentDefaultsHelpFormatter

    arg_parser = argparse.ArgumentParser(formatter_class=fmtr)

    arg_parser.add_argument('-p', '--port', default='/dev/ttyUSB0')

    arg_parser.add_argument('-g', '--gamepad', action='store_true')

    args = arg_parser.parse_args()

    port = Serial(args.port, 115200)

    gamepad_vals = [0, 0, 0, 0, 0, 0]

    msp_parser = MyMspParser(gamepad_vals)

    running = [True]

    gamepad_thread = Thread(target=gamepad_threadfun, args=(gamepad_vals, running))

    prev = time()

    if args.gamepad:

        gamepad_thread.start()

    while True:

        try:

            c = port.read(1)

            msp_parser.parse(c)

        except KeyboardInterrupt:

            running = [False]

            break


main()
