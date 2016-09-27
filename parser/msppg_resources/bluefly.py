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

import pygame
from pygame.locals import KEYDOWN, K_ESCAPE, QUIT
 
class App:

    def __init__(self):

        pygame.init()
 
        if pygame.joystick.get_count() < 1:
            print('No joy')
            exit(1)

        pygame.joystick.init()

        self.joystick_name = pygame.joystick.Joystick(0).get_name()
 
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
 
    def main(self):

        while (True):

            try:

                pygame.event.get()
     
                print('Roll: %+3.3f | Pitch: %+3.3f | Yaw: %+3.3f | Throttle: %+3.3f' % 
                        (self._chanval(0), self._chanval(1), self._chanval(2), self._chanval(3)))

            except KeyboardInterrupt:

                break

    def _chanval(self, index):
        return self.joystick.get_axis(CHANMAP[index])
 
    def quit(self):

        pygame.display.quit()
 
app = App()
app.main()
