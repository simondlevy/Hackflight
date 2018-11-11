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

SUPERFLY_ADDR = '192.168.4.1'
SUPERFLY_PORT = 80
TIMEOUT_SEC   = 4

from socket import socket
from pysticks import get_controller
from msppg import serialize_SET_RC_BYTES

# Start the controller
controller = get_controller()

# Set up socket connection to SuperFly
sock = socket()
sock.settimeout(TIMEOUT_SEC)
sock.connect((SUPERFLY_ADDR, SUPERFLY_PORT))    
    
while True:

    # Make the controller acquire the next event
    controller.update()

    # Put stick demands and aux switch value into a single array
    cmds = [controller.getAxis(k) for k in range(4)] + [controller.getAux()]

    # Conver the array from [-1,+1] to [0,255], and append a 0 for channel 6 for now
    cmds = [int(127*(cmd+1)) for cmd in cmds] + [0]

    print(cmds)

    # Send the array of commands to SuperFly
    sock.send(serialize_SET_RC_BYTES(*cmds))

