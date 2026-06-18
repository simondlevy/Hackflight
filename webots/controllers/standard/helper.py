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

class Helper:

    JOYSTICK_AXIS_MAP = {
        'Logitech Gamepad F310': None,
        'Microsoft X-Box 360 pad': None,
    }

    # These should agree with modes in hackflight/src/datatypes.hpp
    MODES = {'armed': 1, 'hovering': 2, 'autonomous': 3}

    def __init__(self):

        self.robot = Robot()

        self.timestep = int(self.robot.getBasicTimeStep())

        self.joystick = self.robot.getJoystick()

        self.joystick.enable(self.timestep)

        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)

        self.emitter = self.robot.getDevice('emitter')

        self.robot.step(self.timestep)

        self.use_keyboard = False

        if self.joystick.is_connected:

            if self.joystick.model not in self.JOYSTICK_AXIS_MAP:
                print('Unrecognized joystick %s' % self.joystick.model)
                self.use_keyboard = True

        else:
            self.use_keyboard = True

        if self.use_keyboard:
            self.printKeyboardInstructions()

        self.buttons_down = {'hover': False, 'auto': False}

        self.cmdinfo = 'armed', 0, 0, 0, 0

    def startMotor(self, name, direction):

        motor = self.makeMotor(name)
        motor.setVelocity(direction * 60)

    def step(self):

        if self.robot.step(self.timestep) == -1:
            return False

        self.cmdinfo = (self.getCommandInfoFromKeyboard(
           self. keyboard, self.buttons_down, self.cmdinfo)
                   if self.use_keyboard
                   else self.getCommandInfoFromGamepad(
                       self.joystick, self.buttons_down, self.cmdinfo))

        mode = self.cmdinfo[0]

        # Send siminfo to fast thread
        self.emitter.send(struct.pack('Iffff', int(self.MODES[mode]),
                          *self.cmdinfo[1:]))

        return True

    def makeMotor(self, name):

        motor = self.robot.getDevice(name)
        motor.setPosition(float('inf'))
        return motor

    def printKeyboardInstructions(self):
        print('Using keyboard instead:\n')
        print('- Use Enter to take off and land\n')
        print('- Use W and S to go up and down\n')
        print('- Use arrow keys to move horizontally\n')
        print('- Use Q and E to change heading\n')

    def switchMode(self, what, mode):
        '''A little state-transition machine'''
        return (
            'hovering' if mode == 'armed' and what == 'hover' else
            'armed' if mode == 'hovering' and what == 'hover' else
            'autonomous' if mode == 'hovering' and what == 'auto' else
            'hovering' if mode == 'autonomous' and what == 'auto' else
            mode)

    def checkPressed(self, button, target, what, buttons_down, mode):
        if button == target:
            if not buttons_down[what]:
                mode = self.switchMode(what, mode)
            buttons_down[what] = True
        else:
            buttons_down[what] = False
        return mode

    def normalizeJoystickAxis(self, rawval):
        return 2 * rawval / (2**16)

    def readJoystickRaw(self, joystick, index):
        return joystick.getAxisValue(index)

    def readJoystickAxis(self, joystick, index):
        return self.normalizeJoystickAxis(
                self.readJoystickRaw(joystick, index))

    def getMode(self, button, hover_button, auto_button, buttons_down,
                cmdinfo):

        mode = cmdinfo[0]
        mode = self.checkPressed(button, hover_button, 'hover', buttons_down,
                                 mode)
        return self.checkPressed(button, auto_button, 'auto', buttons_down,
                                 mode)

    def getCommandInfoFromGamepad(self, joystick, buttons_down, cmdinfo):

        mode = self.getMode(joystick.getPressedButton(), 4, 5, buttons_down,
                            cmdinfo)

        thrust = -self.readJoystickAxis(joystick, 1)
        roll = self.readJoystickAxis(joystick, 3)
        pitch = -self.readJoystickAxis(joystick, 4)
        yaw = self.readJoystickAxis(joystick, 0)

        return mode, thrust, roll, pitch, yaw

    def getCommandInfoFromKeyboard(self, keyboard, keys_down, cmdinfo):

        key = keyboard.getKey()

        mode = self.getMode(key, 4, 32, keys_down, cmdinfo)

        thrust = +1 if key == ord('W') else -1 if key == ord('S') else 0

        roll = (+1 if key == keyboard.RIGHT else
                -1 if key == keyboard.LEFT else
                0)

        pitch = (+1 if key == keyboard.UP else
                 -1 if key == keyboard.DOWN else
                 0)

        yaw = +0.5 if key == ord('E') else -0.5 if key == ord('Q') else 0

        return mode, thrust, roll, pitch, yaw
