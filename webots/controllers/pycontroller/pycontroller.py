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

import struct

from controller import Robot

JOYSTICK_AXIS_MAP = {
    'Logitech Gamepad F310': (-2, 4, -5, 1),
    'Microsoft X-Box 360 pad': (-2, 4, -5, 1)
}

MODES = {'idle': 0, 'hovering': 2, 'autonomous': 2, 'landing': 3}


def startMotor(robot, motor_name, direction):

    motor = robot.getDevice(motor_name)
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


def checkPressed(button, target, what, buttons_down, mode):
    if button == target:
        if not buttons_down[what]:
            mode = switchMode(what, mode)
        buttons_down[what] = True
    else:
        buttons_down[what] = False
    return mode


def normalizeJoystickAxis(rawval):
    return 2 * rawval / (2**16)


def readJoystickRaw(joystick, index):
    axis = abs(index) - 1
    sign = -1 if index < 0 else +1
    return sign * joystick.getAxisValue(axis)


def readJoystickAxis(joystick, index):
    return normalizeJoystickAxis(readJoystickRaw(joystick, index))


def getModeFromButton(
        button, hover_button, auto_button, buttons_down, cmdinfo):

    mode = cmdinfo[0]
    mode = checkPressed(button, hover_button, 'hover', buttons_down, mode)
    return checkPressed(button, auto_button, 'auto', buttons_down, mode)


def getCommandInfoFromJoystick(joystick, buttons_down, cmdinfo):

    mode = getModeFromButton(joystick.getPressedButton(), 5, 4, buttons_down,
                             cmdinfo)

    axes = JOYSTICK_AXIS_MAP[joystick.model]

    thrust = readJoystickAxis(joystick, axes[0])
    roll = readJoystickAxis(joystick, axes[1])
    pitch = readJoystickAxis(joystick, axes[2])
    yaw = readJoystickAxis(joystick, axes[3])

    return mode, thrust, roll, pitch, yaw


def getCommandInfoFromKeyboard(keyboard, keys_down, cmdinfo):

    key = keyboard.getKey()

    mode = getModeFromButton(key, 4, 32, keys_down, cmdinfo)

    thrust = +1 if key == ord('W') else -1 if key == ord('S') else 0

    roll = (+1 if key == keyboard.RIGHT else
            -1 if key == keyboard.LEFT else
            0)

    pitch = (+1 if key == keyboard.UP else
             -1 if key == keyboard.DOWN else
             0)

    yaw = +0.5 if key == ord('E') else -0.5 if key == ord('Q') else 0

    return mode, thrust, roll, pitch, yaw


def getAndEnableDevice(robot, timestep, device_name):
    device = robot.getDevice(device_name)
    device.enable(timestep)
    return device


def main():

    robot = Robot()

    timestep = int(robot.getBasicTimeStep())

    joystick = robot.getJoystick()
    joystick.enable(timestep)

    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)

    gps = getAndEnableDevice(robot, timestep, 'gps')
    imu = getAndEnableDevice(robot, timestep, 'inertial unit')
    ranger = getAndEnableDevice(robot, timestep, 'range-finder')
    emitter = robot.getDevice('emitter')

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

    xyz = gps.getValues()
    rpy = imu.getRollPitchYaw()

    # Negate for leftward/nose-right positive
    startpose = (xyz[0], -xyz[1], xyz[2], rpy[0], rpy[1], -rpy[2])

    cmdinfo = 'idle', 0, 0, 0, 0

    startMotor(robot, 'motor1', -1)
    startMotor(robot, 'motor2', +1)
    startMotor(robot, 'motor3', +1)
    startMotor(robot, 'motor4', -1)

    while True:

        if robot.step(timestep) == -1:
            break

        cmdinfo = (getCommandInfoFromKeyboard(keyboard, buttons_down, cmdinfo)
                   if use_keyboard
                   else getCommandInfoFromJoystick(
                       joystick, buttons_down, cmdinfo))

        emitter.send(struct.pack(

                'ddddddfIffff',

                # starting pose
                startpose[0], startpose[1], startpose[2],
                startpose[3], startpose[4], startpose[5],

                # framerate
                timestep,

                # mode
                int(MODES[cmdinfo[0]]),

                # setpoint
                cmdinfo[1], cmdinfo[2], cmdinfo[3], cmdinfo[4]))


main()
