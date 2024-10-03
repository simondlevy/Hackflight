#!/usr/bin/python3
'''
Hackflight Ground Control Station

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

from inputs import get_gamepad
from threading import Thread


def threadfun(axis_vals):

    while axis_vals[0] != 9999:

        print(axis_vals)


def main():

    AXIS_MAP = {'X': 0, 'Y': 1, 'Z': 2, 'RX': 3, 'RY': 4, 'RZ':5}

    axis_vals = [0, 1024, 1024, 1024, 1024, 1024]

    thread = Thread(target=threadfun, args=(axis_vals,))

    thread.start()

    while True:

        try:

            for event in get_gamepad():

                code = str(event.code)

                if 'ABS' in code:

                    axis = AXIS_MAP[code[4:]]

                    axis_vals[axis] = event.state

        except KeyboardInterrupt:

            axis_vals[0] = 9999

            break

main()
