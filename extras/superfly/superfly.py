#!/usr/bin/env python3
'''
superfly.py : Python script to fly the SuperFly flight controller over wifi

Requires: pygame

Copyright (C) Simon D. Levy 2016

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

SUPERFLY_ADDR        = '192.168.4.1'
SUPERFLY_PORT        = 80
SUPERFLY_TIMEOUT_SEC = 4

from socket import socket

from pysticks import get_controller

from msppg import serialize_SET_RC_BYTES

from sys import stdout

if __name__ == '__main__':

    controller = get_controller()
        
    while True:

        controller.update()

        for k in range(4):
            stdout.write('%+2.2f ' % controller.getAxis(k))

        stdout.write(' | %d\n' % controller.getAux())
