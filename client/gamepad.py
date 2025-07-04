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

    def threadfun(self, vals, status):

        arming_button_state_prev = 0
        hover_button_state_prev = 0

        while status['running']:

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

                            status['armed'] = not status['armed']

                        arming_button_state_prev = event.state

                    # Hover button
                    elif code == 'BTN_TR':

                        if (status['armed'] and not event.state and
                                hover_button_state_prev):

                            status['hovering'] = not status['hovering']

                        hover_button_state_prev = event.state

            except inputs.UnpluggedError:
                print('No gamepad detected')
                status['running'] = False

            except OSError:
                print('Gamepad unplugged')
                status['running'] = False

    def scale(self, axval):

        return axval / 32767

    def run(self, debug=False):

        gamepads = inputs.devices.gamepads

        if len(gamepads) == 0:
            print('No gamepad detected')
            exit(0)

        status = {'running': True, 'armed': False, 'hovering': False}

        gamepad_vals = [0, 0, 0, 0]

        devname = inputs.devices.gamepads[0].name

        if devname not in self.SUPPORTED:
            print(devname + ' not supported')
            exit(0)

        was_armed = False

        descend_countdown = 0

        zdist = self.ZDIST_INIT

        thread = Thread(target=self.threadfun, args=(gamepad_vals, status))
        thread.daemon = True
        thread.start()

        debug = True

        while status['running']:

            try:

                armed = status['armed']

                if armed != was_armed:
                    #client.send(MspParser.serialize_SET_ARMING(armed))
                    was_armed = armed

                if debug:
                    print('armed=%d' % armed, end=' | ')

                if status['hovering']:

                    descend_countdown = self.THRUST_DESCEND_MAX

                    vx = -self.scale(gamepad_vals[2])  # make forward positive
                    vy = self.scale(gamepad_vals[1])
                    yawrate = self.scale(gamepad_vals[3])

                    t = -self.scale(gamepad_vals[0]) * self.ZDIST_INC

                    zdist = min(max(zdist + t, self.ZDIST_MIN), self.ZDIST_MAX)

                    #client.send(MspParser.serialize_SET_SETPOINT_HOVER(vx, vy, yawrate, zdist))

                    if debug:
                        print(('send_hover_setpoint: vx=%+3.2f vy=%+3.3f ' +
                              'yawrate=%+3.f zdistance=%+3.2f') %
                              (vx, vy, yawrate, zdist))

                else:

                    r = 0
                    p = 0
                    y = 0

                    t = (descend_countdown
                         if descend_countdown > self.THRUST_DESCEND_MIN
                         else 0)

                    descend_countdown -= (self.THRUST_DESCEND_DEC
                                          if descend_countdown > 0 else 0)

                    zdist = self.ZDIST_INIT

                    # client.send(MspParser.serialize_SET_SETPOINT_RPYT(r, p, y, t))

                    if debug:
                        print('send_setpoint: r=%+3.2f p=%+3.3f y=%+3.f t=%d'
                              % (r, p, y, t))

                sleep(1 / self.UPDATE_RATE_HZ)

            except KeyboardInterrupt:
                status['running'] = False


if __name__ == '__main__':

    Gamepad().run(True)
