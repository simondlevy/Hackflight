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
import struct
from time import sleep
from sys import stdout

SUPPORTED_GAMEPADS = {'Microsoft X-Box 360 pad',  'Logitech Gamepad F310'}

RPI_ADDRESS = 'B8:27:EB:3F:AB:47'
RPI_RADIO_PORT = 1
RPI_LOGGING_PORT = 2


def gamepad_threadfun(vals, status):

    GAMEPAD_AXIS_MAP = {'X': 3, 'Y': 0, 'Z': 1, 'RX': 1, 'RY': 2, 'RZ': 2}

    button_state_prev = 0

    while status['running']:

        try:

            for event in inputs.get_gamepad():

                code = str(event.code)

                if 'ABS' in code:

                    subcode = code[4:]

                    if subcode in GAMEPAD_AXIS_MAP:

                        axis = GAMEPAD_AXIS_MAP[subcode]

                        vals[axis] = event.state

                elif code in {'BTN_TR', 'BTN_PINKIE'}:

                    if not event.state and button_state_prev:

                        status['armed'] = not status['armed']

                    button_state_prev = event.state

        except inputs.UnpluggedError:
            print('No gamepad detected')
            status['running'] = False

        except OSError:
            print('Gamepad unplugged')
            status['running'] = False


def radio_threadfun(client, status, chanvals):

    while status['running']:

        msg = struct.pack('hhhhh',
                          chanvals[0], chanvals[1], chanvals[2], chanvals[3],
                          status['armed'])

        try:
            client.send(msg)
        except:
            status['running'] = False
            return

        sleep(0)  # yield


def logging_threadfun(client, status):

    while status['running']:

        try:
            msg = client.recv(48)
        except:
            status['running'] = False
            return

        state = struct.unpack('ffffffffffff', msg)

        print('phi=%+03.3f  theta=%+03.3f  psi=%+03.3f' %
              (state[6], state[8], state[10]), end='\r')

        sleep(0)  # yield


def connect_to_server(port):

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

    print(' connected')

    return client


def main():

    status = {'running': True, 'armed': False}

    gamepad_vals = [0, 0, 0, 0]

    logging_client = connect_to_server(RPI_LOGGING_PORT)
    Thread(target=logging_threadfun, args=(logging_client, status)).start()

    radio_client = connect_to_server(RPI_RADIO_PORT)
    Thread(target=radio_threadfun,
           args=(radio_client, status, gamepad_vals)).start()

    gamepads = inputs.devices.gamepads

    if len(gamepads) == 0:
        print('No gamepad detected')
        exit(0)

    devname = inputs.devices.gamepads[0].name

    if devname not in SUPPORTED_GAMEPADS:
        print(devname + ' not supported')
        exit(0)

    gamepad_thread = Thread(target=gamepad_threadfun,
                            args=(gamepad_vals, status))

    gamepad_thread.daemon = True

    gamepad_thread.start()

    while status['running']:

        sleep(0)

    print('Server quit!')


main()
