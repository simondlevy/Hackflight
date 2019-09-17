#!/usr/bin/env python3
'''
Test script for MSP disarming of board over a socket

Copyright (C) Simon D. Levy, Pep Marti, Juan Gallostra Acin 2018

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.
This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http:#www.gnu.org/licenses/>.
'''

HOST = '137.113.118.68' # thales.cs.wlu.edu
PORT = 20000

from msppg import serialize_SET_ARMED
import socket

if __name__ == "__main__":

    sock = socket.socket()

    print('Attempting to connect to server %s:%d' % (HOST, PORT))

    try:
        sock.connect((HOST, PORT)) # Note tuple!
        print('Connected!')
    except socket.error:
        print('Failed to connect')
        exit(1)

    try:
        sock.send(serialize_SET_ARMED(False))
    except socket.error:
        print('Failed to send')
