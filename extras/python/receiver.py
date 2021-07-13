#!/usr/bin/env python3
'''
Pygame-based R/C receiver simulator

Supports running pygame on main thread and axis retrieval on another

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy

MIT License
'''


import pygame as pg
import numpy as np
from sys import platform

from debugging import debug


def receiver():

    demands = np.zeros(4)

    # Different axis maps for different controllers, OSs
    axis_map = ([1, 2, 3, 0] if platform == 'win32' else [1, 3, 4, 0])

    invert_pitch = -1

    pg.init()
    pg.joystick.init()

    # Initialize a controller or exit if nothing is plugged in
    try:
        js = pg.joystick.Joystick(0)
        if 'SPEKTRUM' in js.get_name():
            axis_map = [1, 2, 3, 0]
            invert_pitch = +1
        else:
            debug('Sorry, only Spektrum RC is currently supported')
            exit(1)
        js.init()
    except pg.error:
        debug('Would you like to buy a controller?')
        exit(1)

    def update():
        '''
        Should be called on main thread
        '''

        # Grab a queue of all actions done
        events = pg.event.get()

        for event in events:

            # If you move the stick, poll all the directions
            # and return it as a tuple
            if event.type == pg.JOYAXISMOTION:

                axes = [js.get_axis(i) for i in range(js.get_numaxes())]

                # Use axis map to go from axes to demands
                demands[1:4] = np.array([axes[axis_map[i]]
                                         for i in range(1, 4)])

                # Adjust for axis direction
                demands[2] *= invert_pitch

                # Map throttle from [-1,+1] to [0,1]
                demands[0] = (axes[axis_map[0]]) + 1 / 2

    def getDemands():
        '''
        Can be called on any thread
        '''

        return demands.copy()

    return update, getDemands


def main():

    updater, getter = receiver()

    while True:

        try:

            updater()

            debug('T: %+3.3f   R: %+3.3f   P: %+3.3f   Y: %+3.3f' %
                  tuple(getter()))

        except KeyboardInterrupt:

            break


if __name__ == '__main__':
    main()
