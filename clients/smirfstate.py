#!/usr/bin/python3

'''
State message logger

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
import struct
import sys
import time

RPI_LOGGING_PORT = 2

RPI_SERVER = '64:B7:08:94:28:76'


def logging_fun(client, status):

    while status['running']:

        try:
            byte = client.recv(1)

            print('x%02X' % ord(byte))

        except Exception as e:
            print('Failed to receiving logging data: ' + str(e))
            status['running'] = False
            return


def connect_to_server(addr, port):

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
                time.sleep(1)

        except KeyboardInterrupt:
            break

    return client


def main():

    client = connect_to_server(RPI_SERVER, RPI_LOGGING_PORT)

    status = {'running': True}

    while status['running']:

        try:

            logging_fun(client, status)

        except KeyboardInterrupt:

            break


main()
