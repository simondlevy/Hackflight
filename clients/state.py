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

import struct
import argparse

from btsupport import connect_to_server

RPI_LOGGING_PORT = 2


def logging_fun(client, status):

    while status['running']:

        try:
            msg = client.recv(48)

        except Exception as e:
            print('Failed to receiving logging data: ' + str(e))
            status['running'] = False
            return

        x, dx, y, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi = (
                struct.unpack('ffffffffffff', msg))

        print(('dx=%+03.2f dy=%+03.2f z=%+03.2f dz=%+03.2f ' +
               'phi=%+5.1f dphi=%+6.1f theta=%+5.1f dtheta=%+6.1f ' +
               'psi=%+5.1f dpsi=%+5.1f') %
              (dx, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi))


def main():

    client = connect_to_server(RPI_LOGGING_PORT)

    status = {'running': True}

    while status['running']:

        try:

            logging_fun(client, status)

        except KeyboardInterrupt:

            break


main()
