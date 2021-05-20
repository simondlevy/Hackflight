"""
Simulated receiver class in python

Adds in a gamecontroller class to allow for xbox one controller
inputs. The getaxes() method returns a tuples with 4 values
in the order: Left Vertical, Right Horizontal, Right Vertical
and Left Vertical.

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy

MIT License
"""


import pygame as pg
# import time
import numpy as np
from sys import platform, stdout


class Receiver(object):
    # Value used to deadband filter the axes
    DEADBAND = 0.25

    def __init__(self):
        """Set up demands and an axis map used to return
        the axis values in correct order"""
        self.demands = np.zeros(4)

        # Different axis maps for Windows and Linux
        self.axis_map = ([1, 2, 3, 0] if platform == "win32" else [1, 3, 4, 0])
        # windows controller map
        # axis[0] -> demands[3]
        # -axis[1] -> demand[0]
        # axis [2] -> demand[1]
        # -axis[3] -> demand[2]

    def begin(self):
        """Initialize the joystick and associated pygame instances"""
        pg.init()
        pg.joystick.init()

        # Initialize a controller or exit if nothing is plugged in
        try:
            self.js = pg.joystick.Joystick(0)
            self.js.init()
        except pg.error:
            self.debug('Would you like to buy a controller?')
            exit(1)

    def getDemands(self):
        """Get and returns the 4 axes positions as a tuple of 4 floats"""
        # Grab a queue of all actions done
        events = pg.event.get()

        for event in events:

            # If you move the stick, poll all the directions
            # and return it as a tuple
            if event.type == pg.JOYAXISMOTION:

                # Apply this deadband filter so that it doesn't just print
                # everything since the sticks don't sit
                # perfectly at 0.0
                axes = [self._deadband(self.js.get_axis(i))
                        for i in range(self.js.get_numaxes())]

                # Use axis map to go from axes to demands
                for i in range(4):
                    self.demands[i] = axes[self.axis_map[i]]

                # Adjust for axis direction
                self.demands[0] *= -1
                self.demands[2] *= -1

        return self.demands

    def _deadband(self, val):
        """Function used to deadband the joystick inputs"""
        return val if abs(val) > self.DEADBAND else 0

    def debug(self, msg):
        """Function used to print things to terminal"""
        print(msg)
        stdout.flush()
