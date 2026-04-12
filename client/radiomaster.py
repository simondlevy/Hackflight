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

import inputs
from threading import Thread
from time import sleep
from sys import stdout


class Gamepad:

    SUPPORTED = {'NATIONS RADIOMASTER SIM'}

    GAMEPAD_AXIS_MAP = {'Z': 0, 'X': 1, 'Y': 2, 'RX': 3}

    UPDATE_RATE_HZ = 100

    ALTITUDE_INIT_M = 0.4
    ALTITUDE_MAX_M = 1.0
    ALTITUDE_MIN_M = 0.2
    ALTITUDE_INC_MPS = 0.01

    def __init__(self, debug=False):

        self.armed = False
        self.debug = debug
        self.connected = True
        self.thrust = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        gamepads = inputs.devices.gamepads

        if len(gamepads) == 0:
            print('No gamepad detected')
            exit(0)

        self.gamepad_vals = [1500] * 4

        devname = inputs.devices.gamepads[0].name

        if devname not in self.SUPPORTED:
            print(devname + ' not supported')
            exit(0)

        thread = Thread(target=self.threadfun, args=(self.gamepad_vals, ))
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

                        if subcode in self.GAMEPAD_AXIS_MAP:

                            axis = self.GAMEPAD_AXIS_MAP[subcode]

                            vals[axis] = event.state

                        elif subcode == 'RY':

                            self.armed = not self.armed

            except inputs.UnpluggedError:
                print('No gamepad detected')
                self.connected = False

            except OSError:
                print('Gamepad unplugged')
                self.connected = False

    def scale(self, axval):

        return 2 * (axval - 989) / 1024 - 1

    def step(self):

        try:

            self.thrust = self.scale(self.gamepad_vals[0])
            self.roll = self.scale(self.gamepad_vals[1])
            self.pitch = self.scale(self.gamepad_vals[2])
            self.yaw = self.scale(self.gamepad_vals[3])

            if self.debug:

                print('armed=%d | t=%3.3f r=%+3.2f p=%+3.3f y=%+3.3f' %
                      (self.armed, self.thrust, self.roll, self.pitch,
                       self.yaw))

            sleep(1 / self.UPDATE_RATE_HZ)

            stdout.flush()

        except KeyboardInterrupt:
            self.connected = False


if __name__ == '__main__':

    gamepad = Gamepad(True)

    while gamepad.connected:

        gamepad.step()
