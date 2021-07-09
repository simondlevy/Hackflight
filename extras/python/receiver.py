#!/usr/bin/env python3
'''
Simulated receiver class in python

Adds in a gamecontroller class to allow for xbox one controller
inputs. The getaxes() method returns a tuples with 4 values
in the order: Left Vertical, Right Horizontal, Right Vertical
and Left Vertical.

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy

MIT License
'''


import pygame as pg
import numpy as np
from sys import platform

from debugging import debug


class Receiver(object):

    def __init__(self, throttle_divisor=100):
        '''Sets up demands and an axis map used to return the axis values in
        correct order For game controllers with springy throttle stick:
        accumulates throttle as you push up, down.'''
        self.demands = np.zeros(4)

        self.throttle_divisor = throttle_divisor

        # Different axis maps for different controllers, OSs
        self.axis_map = None

        # Default axis map and inverters
        self.axis_map = ([1, 2, 3, 0] if platform == 'win32' else [1, 3, 4, 0])
        self.invert_pitch = -1
        self.throttle_fun = Receiver.throttle_fun_game_controller
        self.throttle = 0
        self.throttle_inc = 0

        self.axes = None

    def begin(self):
        '''Initializes the joystick and associated pygame instances'''
        pg.init()
        pg.joystick.init()

        # Initialize a controller or exit if nothing is plugged in
        try:
            self.js = pg.joystick.Joystick(0)
            if 'SPEKTRUM' in self.js.get_name():
                self.axis_map = [1, 2, 3, 0]
                self.invert_pitch = +1
                self.throttle_fun = Receiver.throttle_fun_rc
            self.js.init()
        except pg.error:
            debug('Would you like to buy a controller?')
            exit(1)

    def update(self):
        '''
        Should be called on main thread
        '''

        # Grab a queue of all actions done
        events = pg.event.get()

        for event in events:

            # If you move the stick, poll all the directions
            # and return it as a tuple
            if event.type == pg.JOYAXISMOTION:

                self.axes = [self.js.get_axis(i)
                             for i in range(self.js.get_numaxes())]

                # Use axis map to go from axes to demands
                self.demands[1:4] = np.array([self.axes[self.axis_map[i]]
                                             for i in range(1, 4)])

                # Adjust for axis direction
                self.demands[2] *= self.invert_pitch

        # Use special handling for throttle
        if self.axes is not None:
            self.demands[0] = self.throttle_fun(self,
                                                self.axes[self.axis_map[0]])

    def getDemands(self):
        '''
        Can be called on any thread
        '''

        return self.demands.copy()

    def throttle_fun_rc(self, val):
        return (val + 1) / 2

    def throttle_fun_game_controller(self, val):
        self.throttle_inc = -val / self.throttle_divisor
        self.throttle = np.clip(self.throttle + self.throttle_inc, 0, 1)
        return self.throttle


def main():

    receiver = Receiver(throttle_divisor=10000)

    receiver.begin()

    while True:

        try:

            receiver.update()

            debug('T: %+3.3f   R: %+3.3f   P: %+3.3f   Y: %+3.3f' %
                  tuple(receiver.getDemands()))

        except KeyboardInterrupt:

            break


if __name__ == '__main__':
    main()
