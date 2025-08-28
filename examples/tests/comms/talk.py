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

import argparse
from argparse import ArgumentDefaultsHelpFormatter
import socket
import sys
from time import sleep

BLUETOOTH_ADDRESSES = {
        'onboard': '64:B7:08:94:2A:32',
        'bench': '64:B7:08:93:71:1E'
}

BLUETOOTH_PORT = 1


def connect_to_server(name, port):

    addr = BLUETOOTH_ADDRESSES[name]

    while True:

        try:

            print('Connecting to server %s:%d ... ' % (addr, port), end='')
            sys.stdout.flush()

            # Create a Bluetooth or IP socket depending on address format
            client = (socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                                    socket.BTPROTO_RFCOMM)
                      if ':' in addr
                      else socket.socket(socket.AF_INET, socket.SOCK_STREAM))

            try:
                client.connect((addr, port))
                print(' connected')
                break

            except Exception as e:
                print(str(e) + ': is server running?')
                sleep(1)

        except KeyboardInterrupt:
            break

    return client


def main():

    argparser = argparse.ArgumentParser(
            formatter_class=ArgumentDefaultsHelpFormatter)

    argparser.add_argument('-b', '--bluetooth-server',
                           choices=['onboard', 'bench'],
                           default='onboard', help='Bluetooth server')

    args = argparser.parse_args()

    client = connect_to_server(args.bluetooth_server, BLUETOOTH_PORT)

    while True:

        try:

            msg = input('Say something > ')
            client.send((msg + '\n').encode())

            print(client.recv(100).decode())

        except KeyboardInterrupt:
            break


main()
