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
import inputs
from threading import Thread
import serial
from time import sleep

from msp import Serializer


class RadioMaster:

    SUPPORTED = {'NATIONS RADIOMASTER SIM'}

    AXIS_MAP = {'Z': 0, 'X': 1, 'Y': 2, 'RX': 3, 'RY': 4}

    UPDATE_RATE_HZ = 100

    ALTITUDE_INIT_M = 0.4
    ALTITUDE_MAX_M = 1.0
    ALTITUDE_MIN_M = 0.2
    ALTITUDE_INC_MPS = 0.01

    def __init__(self, port, debug=False):

        self.port = port
        self.debug = debug
        self.connected = True

        gamepads = inputs.devices.gamepads

        if len(gamepads) == 0:
            print('No gamepad detected')
            exit(0)

        # Force neutral axis values to start
        self.axes = [988, 1500, 1500, 1500, 988]

        devname = inputs.devices.gamepads[0].name

        if devname not in self.SUPPORTED:
            print(devname + ' not supported')
            exit(0)

        thread = Thread(target=self.threadfun, args=(self.axes, ))
        thread.daemon = True
        thread.start()

    def threadfun(self, vals):

        while self.connected:

            try:

                for event in inputs.get_gamepad():

                    code = str(event.code)

                    # Axis
                    if 'ABS' in code:

                        subcode = code[4:]

                        if subcode in self.AXIS_MAP:

                            axis = self.AXIS_MAP[subcode]

                            vals[axis] = event.state

            except inputs.UnpluggedError:
                print('No gamepad detected')
                self.connected = False

            except OSError:
                print('RadioMaster unplugged')
                self.connected = False

    def step(self):

        try:

            msg = Serializer.serialize_SET_RC(*self.axes)

            self.port.write(msg)

            for byte in msg:
                print('%d' % byte)
            print()

            sleep(1 / self.UPDATE_RATE_HZ)

            if self.debug:

                print('c1=%04d c2=%04d c3=%04d c4=%04d c5=%04d' %
                      (self.axes[0], self.axes[1], self.axes[2],
                       self.axes[3], self.axes[4]))

        except KeyboardInterrupt:
            self.connected = False


if __name__ == '__main__':

    argparser = argparse.ArgumentParser(
            formatter_class=ArgumentDefaultsHelpFormatter)

    argparser.add_argument('-p', '--port', default='/dev/ttyUSB0',
                           help='Report channel values')

    argparser.add_argument('-d', '--debug', action='store_true',
                           help='Report channel values')

    args = argparser.parse_args()

    try:
        port = serial.Serial(args.port, 115200)

    except serial.SerialException:
        print('Unable to open port ' + args.port)
        exit(0)

    rm = RadioMaster(port, args.debug)

    while rm.connected:

        rm.step()
