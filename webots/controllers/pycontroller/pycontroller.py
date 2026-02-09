#!/usr/bin/env python
'''
   Python flight simulator main for Hackflight with C++ custom physics plugin

   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
'''

from sys import argv

from controller import Robot, Joystick

TIME_STEP = 32

MODE_IDLE =  0
MODE_ARMED = 1
MODE_HOVERING = 2
MODE_AUTONOMOUS = 3
MODE_LANDING = 4
MODE_PANIC = 5

JOYSTICK_AXIS_MAP = {
    'Logitech Gamepad F310': (-2,  4, -5, 1) ,
    'Microsoft X-Box 360 pad': (-2,  4, -5, 1 )
}


def printKeyboardInstructions():
    print('Using keyboard instead:\n');
    print('- Use Enter to take off and land\n');
    print('- Use W and S to go up and down\n');
    print('- Use arrow keys to move horizontally\n');
    print('- Use Q and E to change heading\n');


def reportUnrecognizedJoystick(joystick):
    print('Unrecognized joystick %s with axes ' % joystick.model, end='')
    for k in range(joystick.number_of_axes):
        print('%2d=%+6d |' % (k+1, joystick.getAxisValue(k)), end=' ')
    print()


def main():

    setpointlogfp = open(argv[3], 'w')

    mode = MODE_IDLE

    robot = Robot()

    joystick = robot.getJoystick()

    joystick.enable(TIME_STEP)

    did_warn = False

    while True:

        if robot.step(TIME_STEP) == -1:
            break

        if joystick.is_connected:

            if joystick.model in JOYSTICK_AXIS_MAP:
                print('okay')

            else:
                reportUnrecognizedJoystick(joystick)

        elif not did_warn:
            printKeyboardInstructions()

        did_warn = True

main()
