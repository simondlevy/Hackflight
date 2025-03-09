#!/usr/bin/python3

'''
Hackflight Ground Control Station program

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

import socket
import inputs
from threading import Thread
from struct import unpack
from time import sleep
from sys import stdout

SUPPORTED_GAMEPADS = {'Microsoft X-Box 360 pad',  'Logitech Gamepad F310'}

RPI_ADDRESS = 'B8:27:EB:3F:AB:47'
RPI_RADIO_PORT = 1
RPI_LOGGING_PORT = 2


def gamepad_threadfun(vals, status):

    was_armed = False

    while status['running']:

        # scaled = tuple(map(lambda x: x/32768,
        #                (-vals[0], vals[1], -vals[2], vals[3])))

        # print('t=%+3.3f  r=%+3.3f  p=%+3.3f  y=%+3.3f' % scaled, end=' ')
        # print('Armed: %d' % status['armed'])

        armed = status['armed']

        if armed and not was_armed:
            print('************************* Armed *************************')

        if not armed and was_armed:
            print('*********************** Disarmed ************************')

        was_armed = armed

        sleep(0)  # yield


def logging_threadfun(client, status):

    while status['running']:

        for val in unpack('ffffffffffff', (client.recv(48))):
            print('%+3.3f' % val, end=' ')
        print()

        sleep(0)  # yield


def connect_to_server(port, threadfun, status):

    print('Connecting to server %s:%d ... ' % (RPI_ADDRESS, port), end='')
    stdout.flush()

    # Create a Bluetooth or IP socket depending on address format
    client = (socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
              socket.BTPROTO_RFCOMM)
              if ':' in RPI_ADDRESS
              else socket.socket(socket.AF_INET, socket.SOCK_STREAM))

    try:
        client.connect((RPI_ADDRESS, port))

    except Exception as e:
        print(str(e) + ': is server running?')
        exit(0)

    Thread(target=threadfun, args=(client, status)).start()

    print(' connected')


def main():

    GAMEPAD_AXIS_MAP = {'X': 3, 'Y': 0, 'Z': 1, 'RX': 1, 'RY': 2, 'RZ': 2}

    status = {'running': True, 'armed': False}

    # connect_to_server(RPI_RADIO_PORT, , status)

    connect_to_server(RPI_LOGGING_PORT, logging_threadfun, status)

    gamepads = inputs.devices.gamepads

    if len(gamepads) == 0:
        print('No gamepad detected')
        exit(0)

    devname = inputs.devices.gamepads[0].name

    if devname not in SUPPORTED_GAMEPADS:
        print(devname + ' not supported')
        exit(0)

    gamepad_vals = [0, 0, 0, 0]

    button_state_prev = 0

    Thread(target=gamepad_threadfun, args=(gamepad_vals, status)).start()

    # Gamepad reading blocks, so run it on main thread
    while status['running']:

        try:

            for event in inputs.get_gamepad():

                code = str(event.code)

                if 'ABS' in code:

                    subcode = code[4:]

                    if subcode in GAMEPAD_AXIS_MAP:

                        axis = GAMEPAD_AXIS_MAP[subcode]

                        gamepad_vals[axis] = event.state

                elif code in {'BTN_TR', 'BTN_PINKIE'}:

                    if not event.state and button_state_prev:

                        status['armed'] = not status['armed']

                    button_state_prev = event.state

        except inputs.UnpluggedError:
            print('No gamepad detected')
            status['running'] = False

        except KeyboardInterrupt:
            status['running'] = False

        except OSError:
            print('Gamepad unplugged')
            status['running'] = False


main()
