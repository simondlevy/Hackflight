#!/usr/bin/env python3
'''
Spektrum DSMX => Bluetooth transducer

Copyright (C) Simon D. Levy 2021

MIT License
'''

import pygame
import socket
from time import sleep

from mspparser import MspParser

JSID = 0

BLADDR = '50:02:91:A1:1E:9A'
BLPORT = 1

sock = socket.socket(socket.AF_BLUETOOTH,
                     socket.SOCK_STREAM,
                     socket.BTPROTO_RFCOMM)

sock.connect((BLADDR, BLPORT))

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(JSID)
joystick.init()

while True:

    try:

        '''
        pygame.event.pump()

        t, r, p, y, a1, a2 = (joystick.get_axis(1),
                              joystick.get_axis(2),
                              joystick.get_axis(3),
                              joystick.get_axis(0),
                              joystick.get_axis(7),
                              joystick.get_axis(4))

        print('t=%+3.3f  r=%+3.3f  p=%+3.3f  y=%+3.3f  a1=%+3.3f  a2=%+3.3f' %
              (t, r, p, y, a1, a2))
        '''

        t, r, y, p, a1, a2 = 0.5, 0, 0, 0, 0, 0

        msg = MspParser.serialize_SET_NORMAL_RC(t)

        sock.send(msg)

    except KeyboardInterrupt:
        break
