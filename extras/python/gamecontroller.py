'''
File: gamecontroller.py
Authors: Nobel Manaye, Sujana Basnet, Nicholas Nguyen
Game controller class program
'''

import pygame as pg
import time
from sys import stdout


def debug(msg):
    """Function used to print things to terminal"""

    print(msg)
    stdout.flush()


class GameController(object):

    def __init__(self):

        pg.init()
        pg.joystick.init()

        # Initialize a controller or exit if nothing is plugged in
        try:
            self.js = pg.joystick.Joystick(0)
            self.js.init()
        except pg.error:
            debug('Would you like to buy a controller?')
            exit(1)

    def getAxes(self):
        """Poll all of the control sticks, if they reach beyond
        the deadzone, then it returns a tuple with the 4
        float values of the sticks' positions in order:
        0) Left Vertical
        1) Right Horizontal
        2) Right Vertical
        3) Left Horizontal"""
        # Grab a queue of all actions done
        events = pg.event.get()

        for event in events:
            # If you move the stick, poll all the directions
            # and return it as a tuple
            if event.type == pg.JOYAXISMOTION:

                # Poll the control sticks (not the triggers)
                for i in range(4):

                    # apply this deadzone so that it doesn't just print
                    # everything since the sticks don't sit
                    # perfectly at 0.0
                    if self.js.get_axis(i) >= .25 or \
                            self.js.get_axis(i) <= -.25:

                        return (-self.js.get_axis(1), self.js.get_axis(2),
                                -self.js.get_axis(3), self.js.get_axis(0))
            else:
                return None


def main():

    # initalize the game controller
    g = GameController()

    while True:
        try:
            # Print the value of the tuples if it has something
            joystickTuple = g.getAxes()
            if joystickTuple is not None:
                debug(joystickTuple)
        except KeyboardInterrupt:
            break
        # Sleep so that it doesn't print super fast
        time.sleep(.1)


if __name__ == '__main__':
    main()
