#!/usr/bin/env python3
'''
Python script to fly the SuperFly flight controller over wifi

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
SUPERFLY_TIMEOUT_SEC = 1

from socket import socket

import pygame

from msppg import serialize_SET_RC_BYTES

# A dictionary to support auto-detection of joystick axes: T A E R Aux order, - means inverted
axismaps = {
        'Controller (Rock Candy Gamepad for Xbox 360)' : (-1,  4, -3, 0), 
        '2In1 USB Joystick'                            : (-1,  2, -3, 0),
        'Logitech Extreme 3D'                          : (-2,  0,  1, 3),
        'Frsky Taranis Joystick'                       : ( 0,  1,  2, 5),
        'SPEKTRUM RECEIVER'                            : ( 1,  2,  5, 0)
        }

# Initialize pygame for joystick support
pygame.display.init()
pygame.joystick.init()
controller = pygame.joystick.Joystick(0)
controller.init()

# Get axis map for controller
controller_name = controller.get_name()
if not controller_name in axismaps.keys():
    print('Unrecognized controller: %s' % controller_name)
    exit(1)

print(controller_name)
exit(0)

# Set up socket connection to SuperFly
sock = socket()
sock.settimeout(SUPERFLY_TIMEOUT_SEC)
sock.connect((SUPERFLY_ADDR, SUPERFLY_PORT))

while True:

    # Get next pygame event
    pygame.event.pump()

    #for k in range(controller.get_numaxes()):
    #    stdout.write('%d: %+2.2f ' % (k controller.get_axis(k)))
    #stdout.write(' | ')
    #for k in range(controller.get_numbuttons())
    #stdout.write(': %s \n' % controller.get_name())

    # Send stick commands to SuperFly
    sock.send(serialize_SET_RC_BYTES(1, 2, 3, 4, 5, 6))

