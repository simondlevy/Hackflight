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
import argparse
from argparse import ArgumentDefaultsHelpFormatter

from msp import Parser as MspParser

from btsupport import connect_to_server

RPI_SETPOINT_PORT = 1

SUPPORTED_GAMEPADS = {

        'Microsoft X-Box 360 pad' : {
            'arm': 'WEST',
            'hover': 'TR',
            'axes': {'Y': 0, 'RX': 1, 'RY': 2, 'X': 3},
            'axfun': lambda val : val / 32767
         },

        'Logitech Gamepad F310' : {
            'arm': 'WEST',
            'hover': 'TR',
            'axes': {'Y': 0, 'RX': 1, 'RY': 2, 'X': 3},
            'axfun': lambda val : val / 32767
        },
}

UPDATE_RATE_HZ = 100

THRUST_DESCEND_MAX = 30000
THRUST_DESCEND_MIN = 15000
THRUST_DESCEND_DEC = 208

ZDIST_INIT = 0.4
ZDIST_MAX = 1.0
ZDIST_MIN = 0.2
ZDIST_INC = 0.01


def gamepad_threadfun(dyct, vals, status):

    arming_button_state_prev = 0

    while status['running']:

        try:

            for event in inputs.get_gamepad():

                code = str(event.code)

                # Stick
                if 'ABS' in code:

                    subcode = code[4:]

                    axes = dyct['axes']

                    if subcode in axes:

                        axis = axes[subcode] # GAMEPAD_AXIS_MAP[subcode]

                        vals[axis] = event.state

                # Arming button
                elif code == 'BTN_' + dyct['arm']:

                    if not event.state and arming_button_state_prev:

                        status['armed'] = not status['armed']
                    
                    arming_button_state_prev = event.state

                # Hover button
                elif code == 'BTN_' + dyct['hover']:

                    status['hovering'] = event.state == 1

        except inputs.UnpluggedError:
            print('No gamepad detected')
            status['running'] = False

        except OSError:
            print('Gamepad unplugged')
            status['running'] = False



def main():

    argparser = argparse.ArgumentParser(
            formatter_class=ArgumentDefaultsHelpFormatter)

    argparser.add_argument('-d', '--debug', action='store_true',
            help='debug controller')

    args = argparser.parse_args()

    gamepads = inputs.devices.gamepads

    if len(gamepads) == 0:
        print('No gamepad detected')
        exit(0)

    status = {'running': True, 'armed': False, 'hovering': False}

    gamepad_vals = [0, 0, 0, 0]

    devname = inputs.devices.gamepads[0].name

    if devname not in SUPPORTED_GAMEPADS:
        print(devname + ' not supported')
        exit(0)

    gamepad = SUPPORTED_GAMEPADS[devname]

    axfun = gamepad['axfun']

    gamepad_thread = Thread(target=gamepad_threadfun,
            args=(gamepad, gamepad_vals, status))

    gamepad_thread.daemon = True

    gamepad_thread.start()

    was_armed = False

    descend_countdown = 0

    client = None # connect_to_server(RPI_SETPOINT_PORT)

    zdist = ZDIST_INIT

    while status['running']:

        try:

            armed = status['armed']

            if armed != was_armed:
                #client.send(MspParser.serialize_SET_ARMING(armed))
                was_armed = armed

            if status['hovering']:

                descend_countdown = THRUST_DESCEND_MAX

                vx = -axfun(gamepad_vals[2])
                vy = -axfun(gamepad_vals[1])
                yawrate = 200 * axfun(gamepad_vals[3])

                t = -axfun(gamepad_vals[0]) * ZDIST_INC

                zdist = min(max(zdist + t, ZDIST_MIN), ZDIST_MAX)

                #client.send(MspParser.serialize_SET_SETPOINT_HOVER(vx, vy, yawrate, zdist))

                if args.debug:
                    print(('armed=%d | send_hover_setpoint: ' +
                          'vx=%+3.2f vy=%+3.3f yawrate=%+3.f zdistance=%+3.2f') %
                          (armed, vx, vy, yawrate, zdist))

            else:

                r = 30 * axfun(gamepad_vals[1])
                p = -30 * axfun(gamepad_vals[2])
                y = 200 * axfun(gamepad_vals[3])

                t = (descend_countdown
                     if descend_countdown > THRUST_DESCEND_MIN
                     else 0)

                descend_countdown -= (
                      THRUST_DESCEND_DEC if descend_countdown > 0 else 0)

                zdist = ZDIST_INIT

                #client.send(MspParser.serialize_SET_SETPOINT_RPYT(r, p, y, t))

                if args.debug:
                    print('armed=%d | send_setpoint: r=%+3.2f p=%+3.3f y=%+3.f t=%d' %
                          (armed, r, p, y, t))

            sleep(1 / UPDATE_RATE_HZ)

        except KeyboardInterrupt:
            status['running'] = False


main()
