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

from controller import Robot

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


def start_motor(quad, motor_name, direction):

    motor = quad.getDevice(motor_name)
    motor.setPosition(float('inf'))
    motor.setVelocity(direction * 60)


def printKeyboardInstructions():
    print('Using keyboard instead:\n')
    print('- Use Enter to take off and land\n')
    print('- Use W and S to go up and down\n')
    print('- Use arrow keys to move horizontally\n')
    print('- Use Q and E to change heading\n')


def reportUnrecognizedJoystick(joystick):
    print('Unrecognized joystick %s with axes ' % joystick.model, end='')
    for k in range(joystick.number_of_axes):
        print('%2d=%+6d |' % (k+1, joystick.getAxisValue(k)), end=' ')
    print()


def getSimInfoFromKeyboard(keyboard, mode):
    return None


def getSimInfoFromJoystick(joystick, mode):
    axes = JOYSTICK_AXIS_MAP[joystick.model]
    button = joystick.getPressedButton()
    print(button)
    return None

def getAndEnableDevice(robot, timestep, device_name):
    device = robot.getDevice(device_name)
    device.enable(timestep)
    return device

def main():

    setpointlogfp = open(argv[3], 'w')

    mode = MODE_IDLE

    robot = Robot()

    timestep = int(robot.getBasicTimeStep())

    joystick = robot.getJoystick()

    joystick.enable(timestep)

    gps = getAndEnableDevice(robot, timestep, 'gps')
    img = getAndEnableDevice(robot, timestep, 'inertial unit')
    camera = getAndEnableDevice(robot, timestep, 'camera')
    ranger = getAndEnableDevice(robot, timestep, 'range-finder')

    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)

    robot.step(timestep)

    use_keyboard = False

    if joystick.is_connected:

        if joystick.model not in JOYSTICK_AXIS_MAP:
            print('Unrecognized joystick %s' % joystick.model)
            use_keyboard = True

    else:
        use_keyboard = True

    if use_keyboard:
        printKeyboardInstructions()

    while True:

        if robot.step(timestep) == -1:
            break

        siminfo = (getSimInfoFromKeyboard(keyboard, mode)
                   if use_keyboard
                   else getSimInfoFromJoystick(joystick, mode))


main()
