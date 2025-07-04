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

        gamepads = inputs.devices.gamepads

        if len(gamepads) == 0:
            print('No gamepad detected')
            exit(0)

        self.status = {'running': True, 'armed': False, 'hovering': False}

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

        arming_button_state_prev = 0
        hover_button_state_prev = 0

        while self.status['running']:

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

                        if not event.state and arming_button_state_prev:

                            self.status['armed'] = not self.status['armed']

                        arming_button_state_prev = event.state

                    # Hover button
                    elif code == 'BTN_TR':

                        if (self.status['armed'] and not event.state and
                                hover_button_state_prev):

                            self.status['hovering'] = (
                                    not self.status['hovering'])

                        hover_button_state_prev = event.state

            except inputs.UnpluggedError:
                print('No gamepad detected')
                self.status['running'] = False

            except OSError:
                print('Gamepad unplugged')
                self.status['running'] = False

    def scale(self, axval):

        return axval / 32767

    def step(self):

        try:

            self.armed = self.status['armed']

            if self.debug:
                print('armed=%d' % self.armed, end=' | ')

            self.hovering = self.status['hovering']

            if self.hovering:

                self.descend_countdown = self.THRUST_DESCEND_MAX

                vx = -self.scale(self.gamepad_vals[2])  # forward positive
                vy = self.scale(self.gamepad_vals[1])
                yawrate = self.scale(self.gamepad_vals[3])

                t = -self.scale(self.gamepad_vals[0]) * self.ZDIST_INC

                self.zdist = min(max(self.zdist + t, self.ZDIST_MIN),
                                 self.ZDIST_MAX)

                if self.debug:
                    print(('send_hover_setpoint: vx=%+3.2f vy=%+3.3f ' +
                          'yawrate=%+3.f self.zdistance=%+3.2f') %
                          (vx, vy, yawrate, self.zdist))

            else:

                r = 0
                p = 0
                y = 0

                t = (self.descend_countdown
                     if self.descend_countdown > self.THRUST_DESCEND_MIN
                     else 0)

                self.descend_countdown -= (self.THRUST_DESCEND_DEC
                                           if self.descend_countdown > 0
                                           else 0)

                self.zdist = self.ZDIST_INIT

                if self.debug:
                    print('send_setpoint: r=%+3.2f p=%+3.3f y=%+3.f t=%d'
                          % (r, p, y, t))

            sleep(1 / self.UPDATE_RATE_HZ)

        except KeyboardInterrupt:
            self.status['running'] = False

        return self.armed, self.hovering


if __name__ == '__main__':

    gamepad = Gamepad(True)

    while gamepad.status['running']:

        gamepad.step()
