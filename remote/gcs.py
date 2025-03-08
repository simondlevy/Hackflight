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
import argparse
import inputs
from threading import Thread
from struct import unpack
from time import sleep
from sys import stdout
import inputs

SUPPORTED_GAMEPADS = { 'Microsoft X-Box 360 pad',  'Logitech Gamepad F310' }


def gamepad_threadfun(vals, status):

    was_armed = False

    while status['running']:

        scaled = tuple(map(lambda x : x/32768,
                          (-vals[0], vals[1], -vals[2], vals[3])))

        # print('t=%+3.3f  r=%+3.3f  p=%+3.3f  y=%+3.3f' % scaled, end=' ')
        # print('Armed: %d' % status['armed'])

        armed = status['armed']

        if armed and not was_armed:
            print('Armed')

        if not armed and was_armed:
            print('Disarmed')

        was_armed = armed

        sleep(0)  # yield


def logging_threadfun(client, status):

    while status['running']:

        for val in unpack('ffffffffffff', (client.recv(48))):
            print('%+3.3f' % val, end=' ')
        print()

        sleep(0)  # yield


def main():

    GAMEPAD_AXIS_MAP = {'X': 3, 'Y': 0, 'Z': 1, 'RX': 1, 'RY': 2, 'RZ': 2}

    # Parse command-line arguments
    parser = argparse.ArgumentParser(
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-a', '--address', default='B8:27:EB:3F:AB:47',
                        help='server address (IP or MAC)')
    parser.add_argument('-p', '--port', help='port', type=int, default=1)
    args = parser.parse_args()

    print('Connecting to server %s:%d ...' % (args.address, args.port), end='')
    stdout.flush()

    # Create a Bluetooth or IP socket depending on address format
    client = (socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                    socket.BTPROTO_RFCOMM)
              if ':' in args.address 
              else socket.socket(socket.AF_INET, socket.SOCK_STREAM))

    try:
        client.connect((args.address, args.port))

    except:
        print('Connection failed; make sure server is running')
        exit(0)

    print(' connected')

    status = {'running': True, 'armed': False}

    Thread(target=logging_threadfun, args=(client, status)).start()

    gamepads = inputs.devices.gamepads

    if len(gamepads) == 0:
        print('No gamepad detected')
        exit(0)
        
    devname = inputs.devices.gamepads[0].name 

    if not devname in SUPPORTED_GAMEPADS:
        print(devname + ' not supported')
        exit(0)

    gamepad_vals = [0, 0, 0, 0]

    button_state_prev = 0

    Thread(target=gamepad_threadfun, args=(gamepad_vals, status)).start()

    while status['running']:

        try:

            for event in inputs.get_gamepad():

                code = str(event.code)

                if 'ABS' in code:

                    subcode = code[4:]

                    if subcode in GAMEPAD_AXIS_MAP:

                        axis = GAMEPAD_AXIS_MAP[subcode]

                        gamepad_vals[axis] = event.state

                elif code in {'BTN_TR', 'BTN_PINKIE'} :

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
