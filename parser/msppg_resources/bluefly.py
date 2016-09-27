#!/usr/bin/env python3

'''
bluefly.py : Fly your quadcopter from a game controller using Bluetooth + PyGame + MSPPG

**************************** BE CAREFUL ********************************

Copyright (C) Simon D. Levy 2016

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.
This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http:#www.gnu.org/licenses/>.
'''

# MAC address of your Bluetooth adapter
BT_ADDR = "00:06:66:73:e3:a6"

# Map from controller axes to RC channels: Roll, Pitch, Yaw, Throttle
CHANMAP = [2,3,0,1]

# Special handling for throttle: starting at zero, then a tiny increment up or down
THROTINC = 1e-5

# PWM range
PWMMIN = 1000
PWMMAX = 2000

import pygame

from msppg import serialize_SET_RAW_RC
 
class App(object):

    def __init__(self):

        pygame.init()
 
        if pygame.joystick.get_count() < 1:
            print('No joy')
            exit(1)

        pygame.joystick.init()

        self.joystick_name = pygame.joystick.Joystick(0).get_name()
 
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.throttle = -1
 
    def main(self):

        while (True):

            try:

                pygame.event.get()

                # Special handling for throttle
                self.throttle -= self._axval(3) * THROTINC
                self.throttle = max(min(self.throttle, +1), -1)
     
                print('Roll: %4d | Pitch: %4d | Yaw: %4d | Throttle: %4d' % 
                        (self._axis2pwm(0), self._axis2pwm(1,-1), self._axis2pwm(2), self._axval2pwm(self.throttle)))

            except KeyboardInterrupt:

                break

    def _axval(self, index):

        return self.joystick.get_axis(CHANMAP[index])

    def _axval2pwm(self, axval):

        c = axval
        a = -1
        b = +1
        z = PWMMAX
        y = PWMMIN

        return int((c - a) * (z - y) / (b - a) + y)

    def _axis2pwm(self, index, sgn=+1):

        return self._axval2pwm(sgn*self._axval(index))
 
app = App()

app.main()
