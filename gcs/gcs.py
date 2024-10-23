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
import inputs
from threading import Thread
from time import sleep

from msp import Parser


class MyMspParser(Parser):

    def __init__(self, gamepad_vals):

        Parser.__init__(self)

        self.gamepad_vals = gamepad_vals

    def handle_STATE(self, dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi):

        print('phi=%+03.0f theta=%+03.0f psi=%+03.0f' % (phi, theta, psi))


def telemetry_threadfun(port, msp, running):

    while running[0]:

        msp.parse(port.read())

        sleep(0)


def gamepad_threadfun(port, msp, vals, running):

    while running[0]:

        msg = msp.serialize_SET_RAW_RC(
                vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]) 

        port.write(msg)

        sleep(0)


def main():

    AXIS_MAP = {'X': 0, 'Y': 1, 'Z': 2, 'RX': 3, 'RY': 4, 'RZ': 5}

    fmtr = argparse.ArgumentDefaultsHelpFormatter

    arg_parser = argparse.ArgumentParser(formatter_class=fmtr)

    arg_parser.add_argument('-p', '--port', default='/dev/ttyUSB0')

    args = arg_parser.parse_args()

    port = Serial(args.port, 115200)

    gamepad_vals = [0, 1024, 1024, 1024, 0, 0]

    msp = MyMspParser(gamepad_vals)

    running = [True]

    t1 = Thread(target=telemetry_threadfun,
           args=(port, msp, running)).start()

    t2 = Thread(target=gamepad_threadfun,
           args=(port, msp, gamepad_vals, running)).start()

    while True:

        try:

            for event in inputs.get_gamepad():

                code = str(event.code)

                if 'ABS' in code:

                    axis = AXIS_MAP[code[4:]]

                    gamepad_vals[axis] = event.state

        except inputs.UnpluggedError:

            print('No gamepad detected')

            break

        except KeyboardInterrupt:

            break

        except OSError:

            print('Gamepad unplugged')

            break

    running[0] = False

main()
