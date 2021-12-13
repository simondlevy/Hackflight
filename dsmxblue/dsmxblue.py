#!/usr/bin/env python3
'''
Spektrum DSMX => Bluetooth transducer

Copyright (C) Simon D. Levy 2021

MIT License
'''

import pygame

JSID = 0

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(JSID)
joystick.init()
joystick.get_axis(JSID)

while True:

    try:
        pygame.event.pump()
        print(joystick.get_axis(0))
    except KeyboardInterrupt:
        break
