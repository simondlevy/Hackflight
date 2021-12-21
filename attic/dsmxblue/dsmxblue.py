#!/usr/bin/env python3
'''
Spektrum DSMX => Bluetooth transducer

Copyright (C) Simon D. Levy 2021

MIT License
'''

import pygame
import socket
from time import sleep
from sys import stdout

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

def scale(axis):

    return int(1000 * (1 + (joystick.get_axis(axis)+1)/2))

while True:

    try:

        pygame.event.pump()

        t, r, p, y, a1, a2 = scale(1), scale(2), scale(3), scale(0), scale(7), scale(4)

        # print('t=%4d  r=%4d  p=%4d  y=%4d  a1=%4d  a2=%4d'% (t, r, p, y, a1, a2))

        msg = MspParser.serialize_SET_RC(t, r, p, y, a1, a2)

        sock.send(msg)

        sleep(.005)

    except KeyboardInterrupt:
        break
