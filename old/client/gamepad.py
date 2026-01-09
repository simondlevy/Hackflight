#!/usr/bin/python3

'''
Copyright (C) 2025 Simon D. Levy

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


class Gamepad:

    SUPPORTED = {'Microsoft X-Box 360 pad', 'Logitech Gamepad F310'}

    GAMEPAD_AXIS_MAP = {'X': 3, 'Y': 0, 'RX': 1, 'RY': 2}

    UPDATE_RATE_HZ = 100

    THRUST_DESCEND_MAX = 30000
    THRUST_DESCEND_MIN = 15000
    THRUST_DESCEND_DEC = 208

    ZDIST_INIT = 0.4
    ZDIST_MAX = 1.0
    ZDIST_MIN = 0.2
    ZDIST_INC = 0.01

    def __init__(self, debug=False):

        self.armed = False
        self.hovering = False
        self.debug = debug
        self.running = True
        self.vx = 0
        self.vy = 0
        self.yawrate = 0
        self.zdist = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.thrust = 0

        gamepads = inputs.devices.gamepads

        if len(gamepads) == 0:
            print('No gamepad detected')
            exit(0)

        self.gamepad_vals = [0, 0, 0, 0]

        devname = inputs.devices.gamepads[0].name

        if devname not in self.SUPPORTED:
            print(devname + ' not supported')
            exit(0)

        self.descend_countdown = 0

        self.zdist = self.ZDIST_INIT

        thread = Thread(target=self.threadfun, args=(self.gamepad_vals, ))
        thread.daemon = True
        thread.start()

    def threadfun(self, vals):

        arming_prev = 0
        hover_prev = 0

        while self.running:

            try:

                for event in inputs.get_gamepad():

                    code = str(event.code)

                    # Stick
                    if 'ABS' in code:

                        subcode = code[4:]

                        if subcode in self.GAMEPAD_AXIS_MAP:

                            axis = self.GAMEPAD_AXIS_MAP[subcode]

                            vals[axis] = event.state

                    # Arming button
                    elif code == 'BTN_WEST':

                        if not event.state and arming_prev:

                            self.armed = not self.armed

                        arming_prev = event.state

                    # Hover button
                    elif code == 'BTN_TR':

                        if self.armed and not event.state and hover_prev:

                            self.hovering = not self.hovering

                        hover_prev = event.state

            except inputs.UnpluggedError:
                print('No gamepad detected')
                self.running = False

            except OSError:
                print('Gamepad unplugged')
                self.running = False

    def scale(self, axval):

        return axval / 32767

    def step(self):

        try:

            if self.debug:
                print('armed=%d' % self.armed, end=' | ')

            if self.hovering:

                self.descend_countdown = self.THRUST_DESCEND_MAX

                self.vx = -self.scale(self.gamepad_vals[2])  # forward positive
                self.vy = self.scale(self.gamepad_vals[1])
                self.yawrate = self.scale(self.gamepad_vals[3])

                self.thrust = (
                        -self.scale(self.gamepad_vals[0]) * self.ZDIST_INC)

                self.zdist = min(max(self.zdist + self.thrust, self.ZDIST_MIN),
                                 self.ZDIST_MAX)

                if self.debug:
                    print(('send_hover_setpoint: vx=%+3.2f vy=%+3.3f ' +
                          'yawrate=%+3.f self.zdistance=%+3.2f') %
                          (self.vx, self.vy, self.yawrate, self.zdist))

            else:

                self.roll = 0
                self.pitch = 0
                self.yaw = 0

                self.thrust = (
                        self.descend_countdown
                        if self.descend_countdown > self.THRUST_DESCEND_MIN
                        else 0)

                self.descend_countdown -= (self.THRUST_DESCEND_DEC
                                           if self.descend_countdown > 0
                                           else 0)

                self.zdist = self.ZDIST_INIT

                if self.debug:
                    print('send_setpoint: r=%+3.2f p=%+3.3f y=%+3.f t=%d'
                          % (self.roll, self.pitch, self.yaw, self.thrust))

            sleep(1 / self.UPDATE_RATE_HZ)

        except KeyboardInterrupt:
            self.running = False


if __name__ == '__main__':

    gamepad = Gamepad(True)

    while gamepad.running:

        gamepad.step()
