#!/usr/bin/python3

'''
Hackflight logigng program

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
from struct import unpack
from time import sleep


def main():

    # Parse command-line arguments
    parser = argparse.ArgumentParser(
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-s', '--server', help='server', default='localhost')
    parser.add_argument('-p', '--port', help='port', type=int, default=9000)
    args = parser.parse_args()

    # Attempt to connect to the server until connection is made
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while True:
        try:
            client.connect((args.server, args.port))
            break
        except Exception:
            print('Waiting for server %s:%d to start' %
                  (args.server, args.port))
            sleep(1)

    # Loop until server quits
    while True:

        try:
            for val in unpack('ffffffffffff', (client.recv(48))):
                print('%+3.3f' % val, end=' ')
            print()

        except KeyboardInterrupt:
            break



main()
