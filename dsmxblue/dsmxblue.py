#!/usr/bin/env python3
'''
Spektrum DSMX => Bluetooth transducer

Copyright (C) Simon D. Levy 2021

MIT License
'''

import pygame
import socket

JSID = 0

BLADDR = '50:02:91:A1:1E:9A'
BLPORT = 1

sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

sock.connect((BLADDR, BLPORT))

while True:

    sock.send(input('Send something > ').encode('utf8'))

# Axes: 0=yaw 1=throttle 2=roll 4=aux1 5=pitch

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(JSID)
joystick.init()
joystick.get_axis(JSID)

while True:

    try:
        pygame.event.pump()
        print(joystick.get_axis(5))
    except KeyboardInterrupt:
        break
