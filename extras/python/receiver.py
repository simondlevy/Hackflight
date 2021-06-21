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


class Receiver(object):

    def __init__(self):
        '''Sets up demands and an axis map used to return
        the axis values in correct order'''
        self.demands = np.zeros(4)

        # Different axis maps for different controllers, OSs
        self.axis_map = None

    def begin(self):
        '''Initializes the joystick and associated pygame instances'''
        pg.init()
        pg.joystick.init()

        # Default axis map and inverters
        self.axis_map = ([1, 2, 3, 0] if platform == 'win32' else [1, 3, 4, 0])
        self.invert = [-1, -1]

        # Initialize a controller or exit if nothing is plugged in
        try:
            self.js = pg.joystick.Joystick(0)
            if 'SPEKTRUM' in self.js.get_name():
                self.axis_map = ([1, 2, 5, 0]if platform == 'win32'
                                 else [1, 2, 3, 0])
                self.invert = [+1, +1]
            self.js.init()
        except pg.error:
            print('Would you like to buy a controller?')
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

                axes = [self.js.get_axis(i)
                        for i in range(self.js.get_numaxes())]

                print('%+3.3f' % axes[3])

                # Use axis map to go from axes to demands
                self.demands = np.array([axes[self.axis_map[i]]
                                         for i in range(4)])

                # Adjust for axis direction
                self.demands[0] *= self.invert[0]
                self.demands[2] *= self.invert[1]

    def getDemands(self):
        '''
        Can be called on any thread
        '''

        return self.demands.copy()


def main():

    receiver = Receiver()

    receiver.begin()

    while True:

        try:

            receiver.update()

            print('T: %+3.3f   R: %+3.3f   P: %+3.3f   Y: %+3.3f' %
                  tuple(receiver.getDemands()))

        except KeyboardInterrupt:

            break


if __name__ == '__main__':
    main()
