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

JOYSTICK_AXIS_MAP = {
    'Logitech Gamepad F310': (-2, 4, -5, 1),
    'Microsoft X-Box 360 pad': (-2, 4, -5, 1)
}

MODES = {'idle': 0, 'hovering': 2, 'autonomous': 2, 'landing': 3}

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


def switchMode(what, mode):
    return (
        'hovering' if mode == 'idle' and what == 'hover' else
        'landing' if mode == 'hovering' and what == 'hover' else
        'autonomous' if mode == 'hovering' and what == 'auto' else
        'hovering' if mode == 'autonomous' and what == 'auto' else
        mode)


def checkButton(button, target, what, buttons_down, mode):
    if button == target:
        if not buttons_down[what]:
            mode = switchMode(what, mode)
        buttons_down[what] = True
    else:
        buttons_down[what] = False
    return mode


def getSimInfoFromKeyboard(keyboard, mode, buttons_down):
    return None


def normalizeJoystickAxis(rawval):
    return 2 * rawval / (2**16)


def readJoystickRaw(joystick, index):
    axis = abs(index) - 1
    sign = -1 if index < 0 else +1
    return sign * joystick.getAxisValue(axis)


def readJoystickAxis(joystick, index):
    return normalizeJoystickAxis(readJoystickRaw(joystick, index))


def getSimInfoFromJoystick(joystick, buttons_down, siminfo):

    button = joystick.getPressedButton()
    mode = siminfo['mode']
    mode = checkButton(button, 5, 'hover', buttons_down, mode)
    mode = checkButton(button, 4, 'auto', buttons_down, mode)
    siminfo['mode'] = mode

    axes = JOYSTICK_AXIS_MAP[joystick.model]
    setpoint = siminfo['setpoint']

    setpoint['thrust'] = readJoystickAxis(joystick, axes[0])
    setpoint['roll'] = readJoystickAxis(joystick, axes[1])
    setpoint['pitch'] = readJoystickAxis(joystick, axes[2])
    setpoint['yaw'] = readJoystickAxis(joystick, axes[3])


def getAndEnableDevice(robot, timestep, device_name):
    device = robot.getDevice(device_name)
    device.enable(timestep)
    return device


def main():

    logfile = open(argv[3], 'w')

    robot = Robot()

    timestep = int(robot.getBasicTimeStep())

    joystick = robot.getJoystick()

    joystick.enable(timestep)

    gps = getAndEnableDevice(robot, timestep, 'gps')
    imu = getAndEnableDevice(robot, timestep, 'inertial unit')
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

    buttons_down = {'hover': False, 'auto': False}

    siminfo = {'mode': 'idle',
               'setpoint': {'thrust': 0, 'roll': 0, 'pitch': 0, 'yaw': 0}}

    xyz = gps.getValues()
    rpy = imu.getRollPitchYaw()

    # Negate for leftward/nose-right positive
    startingPose = (xyz[0], -xyz[1], xyz[2], rpy[0], rpy[1], -rpy[2])

    while True:

        if robot.step(timestep) == -1:
            break

        getSimInfoFromJoystick(joystick, buttons_down, siminfo)

        s = siminfo['setpoint']
        logfile.write('%d,%f,%f,%f,%f\n' % (MODES[siminfo['mode']],
               s['thrust'], s['roll'], s['pitch'], s['yaw']))
        logfile.flush()

        '''
        siminfo = (getSimInfoFromKeyboard(keyboard, mode)
                   if use_keyboard
                   else getSimInfoFromJoystick(joystick, mode, buttons_down))
        '''


main()
