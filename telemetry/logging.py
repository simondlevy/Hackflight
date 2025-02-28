#!/usr/bin/python3

'''
Hackflight logging program

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
    parser.add_argument('-a', '--address', help='server address (IP or MAC)')
    parser.add_argument('-p', '--port', help='port', type=int)
    args = parser.parse_args()

    # Create a Bluetooth or IP socket depending on address format
    client = (socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                    socket.BTPROTO_RFCOMM)
              if ':' in args.address 
              else socket.socket(socket.AF_INET, socket.SOCK_STREAM))

    # Attempt to connect to the server until connection is made
    while True:
        try:
            client.connect((args.address, args.port))
            break
        except Exception:
            print('Waiting for server %s:%d to start' %
                  (args.address, args.port))
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
