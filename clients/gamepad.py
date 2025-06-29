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

from btsupport import connect_to_server

try:
    from msp import Parser as MspParser
except Exception as e:
    print('Install msp: cd ../msppg; make install')
    exit(0)

RPI_SETPOINT_PORT = 1

SUPPORTED_GAMEPADS = {'Microsoft X-Box 360 pad', 'Logitech Gamepad F310'}

GAMEPAD_AXIS_MAP = {'X': 3, 'Y': 0, 'RX': 1, 'RY': 2}

UPDATE_RATE_HZ = 100

THRUST_DESCEND_MAX = 30000
THRUST_DESCEND_MIN = 15000
THRUST_DESCEND_DEC = 208

ZDIST_INIT = 0.4
ZDIST_MAX = 1.0
ZDIST_MIN = 0.2
ZDIST_INC = 0.01


def gamepad_threadfun(vals, status):

    arming_button_state_prev = 0
    hover_button_state_prev = 0

    while status['running']:

        try:

            for event in inputs.get_gamepad():

                code = str(event.code)

                # Stick
                if 'ABS' in code:

                    subcode = code[4:]

                    if subcode in GAMEPAD_AXIS_MAP:

                        axis = GAMEPAD_AXIS_MAP[subcode]

                        vals[axis] = event.state

                # Arming button
                elif code == 'BTN_WEST':

                    if not event.state and arming_button_state_prev:

                        status['armed'] = not status['armed']

                    arming_button_state_prev = event.state

                # Hover button
                elif code == 'BTN_TR':

                    if (status['armed'] and not event.state and hover_button_state_prev):

                        status['hovering'] = not status['hovering']

                    hover_button_state_prev = event.state

        except inputs.UnpluggedError:
            print('No gamepad detected')
            status['running'] = False

        except OSError:
            print('Gamepad unplugged')
            status['running'] = False


def scale(axval):

    return axval / 32767


def main():

    argparser = argparse.ArgumentParser(
            formatter_class=ArgumentDefaultsHelpFormatter)

    argparser.add_argument('-s', '--server',
                           choices=['onboard', 'pihat', 'none'],
                           default='onboard',
                           help='RaspberryPi Bluetooth server')

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

    gamepad_thread = Thread(target=gamepad_threadfun,
                            args=(gamepad_vals, status))

    gamepad_thread.daemon = True

    gamepad_thread.start()

    was_armed = False

    descend_countdown = 0

    if args.server != 'none':
        client = connect_to_server(args.server, RPI_SETPOINT_PORT)

    zdist = ZDIST_INIT

    while status['running']:

        try:

            armed = status['armed']

            if armed != was_armed:
                if args.server != 'none':
                    client.send(MspParser.serialize_SET_ARMING(armed))
                was_armed = armed

            if args.debug:

                print('armed=%d' % armed, end=' | ')

            if status['hovering']:

                descend_countdown = THRUST_DESCEND_MAX

                vx = -scale(gamepad_vals[2])  # negate for forward positive
                vy = scale(gamepad_vals[1])
                yawrate = scale(gamepad_vals[3])

                t = -scale(gamepad_vals[0]) * ZDIST_INC

                zdist = min(max(zdist + t, ZDIST_MIN), ZDIST_MAX)

                if args.server != 'none':
                    client.send(MspParser.serialize_SET_SETPOINT_HOVER(
                        vx, vy, yawrate, zdist))

                if args.debug:
                    print(('send_hover_setpoint: vx=%+3.2f vy=%+3.3f ' +
                           'yawrate=%+3.f zdistance=%+3.2f') %
                          (vx, vy, yawrate, zdist))

            else:

                r = 0
                p = 0
                y = 0

                t = (descend_countdown
                     if descend_countdown > THRUST_DESCEND_MIN
                     else 0)

                descend_countdown -= (
                      THRUST_DESCEND_DEC if descend_countdown > 0 else 0)

                zdist = ZDIST_INIT

                if args.server != 'none':
                    client.send(
                            MspParser.serialize_SET_SETPOINT_RPYT(r, p, y, t))

                if args.debug:
                    print('send_setpoint: r=%+3.2f p=%+3.3f y=%+3.f t=%d' %
                          (r, p, y, t))

            sleep(1 / UPDATE_RATE_HZ)

        except KeyboardInterrupt:
            status['running'] = False


main()
