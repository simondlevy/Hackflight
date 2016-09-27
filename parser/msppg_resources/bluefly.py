#!/usr/bin/env python3

'''
bluefly.py : Fly your quadcopter from a game controller using Bluetooth + PyGame + MSPPG

 Adapted from http://pygame.org/wiki/Joystick_analyzer

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

        pygame.display.set_caption("Bluefly")
 
        # Set up the joystick

        pygame.joystick.init()

        self.my_joystick = None
 
        self.joystick_name = pygame.joystick.Joystick(0).get_name()
 
        self.my_joystick = pygame.joystick.Joystick(0)
        self.my_joystick.init()
 
        max_joy = max(self.my_joystick.get_numaxes(), 
                      self.my_joystick.get_numbuttons(), 
                      self.my_joystick.get_numhats())
 
        self.screen = pygame.display.set_mode( (max_joy * 30 + 10, 170) )
 
        self.font = pygame.font.SysFont("Courier", 20)
 
    # A couple of joystick functions...
    def check_axis(self, p_axis):
        if (self.my_joystick):
            if (p_axis < self.my_joystick.get_numaxes()):
                return self.my_joystick.get_axis(p_axis)
 
        return 0
 
    def check_button(self, p_button):
        if (self.my_joystick):
            if (p_button < self.my_joystick.get_numbuttons()):
                return self.my_joystick.get_button(p_button)
 
        return False
 
    def check_hat(self, p_hat):
        if (self.my_joystick):
            if (p_hat < self.my_joystick.get_numhats()):
                return self.my_joystick.get_hat(p_hat)
 
        return (0, 0)
 
    def draw_text(self, text, x, y, color, align_right=False):
        surface = self.font.render(text, True, color, (0, 0, 0))
        surface.set_colorkey( (0, 0, 0) )
 
        self.screen.blit(surface, (x, y))
 
    def center_text(self, text, x, y, color):
        surface = self.font.render(text, True, color, (0, 0, 0))
        surface.set_colorkey( (0, 0, 0) )
 
        self.screen.blit(surface, (x - surface.get_width() / 2, 
                                   y - surface.get_height() / 2))
 
    def main(self):
        while (True):
            self.g_keys = pygame.event.get()
 
            self.screen.fill(0)
 
            for event in self.g_keys:
                if (event.type == KEYDOWN and event.key == K_ESCAPE):
                    self.quit()
                    return
 
                elif (event.type == QUIT):
                    self.quit()
                    return
 
            self.draw_text("Joystick Name:  %s" % self.joystick_name, 5, 5, (0, 255, 0))
 
            self.draw_text("Axes (%d)" % self.my_joystick.get_numaxes(), 
                           5, 25, (255, 255, 255))
 
            print('Roll: %+3.3f | Pitch: %+3.3f | Yaw: %+3.3f | Throttle: %+3.3f' % 
                    (self._chanval(0), self._chanval(1), self._chanval(2), self._chanval(3)))

            for i in range(0, self.my_joystick.get_numaxes()):
                axval = self.my_joystick.get_axis(i)
                if axval:
                    pygame.draw.circle(self.screen, (0, 0, 200), (20 + (i * 30), 50), 10, 0)
                else:
                    pygame.draw.circle(self.screen, (255, 0, 0), (20 + (i * 30), 50), 10, 0)
 
                self.center_text("%d" % i, 20 + (i * 30), 50, (255, 255, 255))
 
            self.draw_text("Buttons (%d)" % self.my_joystick.get_numbuttons(), 
                           5, 75, (255, 255, 255))
 
            for i in range(0, self.my_joystick.get_numbuttons()):
                if (self.my_joystick.get_button(i)):
                    pygame.draw.circle(self.screen, (0, 0, 200), (20 + (i * 30), 100), 10, 0)
                else:
                    pygame.draw.circle(self.screen, (255, 0, 0), (20 + (i * 30), 100), 10, 0)
 
                self.center_text("%d" % i, 20 + (i * 30), 100, (255, 255, 255))
 
            self.draw_text("POV Hats (%d)" % self.my_joystick.get_numhats(), 
                           5, 125, (255, 255, 255))
 
            for i in range(0, self.my_joystick.get_numhats()):
                if (self.my_joystick.get_hat(i) != (0, 0)):
                    pygame.draw.circle(self.screen, (0, 0, 200), (20 + (i * 30), 150), 10, 0)
                else:
                    pygame.draw.circle(self.screen, (255, 0, 0), (20 + (i * 30), 150), 10, 0)
 
                self.center_text("%d" % i, 20 + (i * 30), 100, (255, 255, 255))
 
            pygame.display.flip()

    def _chanval(self, index):
        return self.my_joystick.get_axis(CHANMAP[index])
 
    def quit(self):
        pygame.display.quit()
 
app = App()
app.main()
