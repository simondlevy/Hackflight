#!/usr/bin/env python3
'''
pysticks.py: Python classes for flying with joysticks, R/C controllers

Requires: pygame

Copyright (C) Simon D. Levy 2016

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify
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

import pygame

class Controller(object):

    STICK_DEADBAND = .05

    def __init__(self, axis_map):

        self.joystick = None
        self.axis_map = axis_map
        
    def update(self):

        pygame.event.pump()

    def getThrottle(self):

        return self._getAxis(0)

    def getRoll(self):

        return self._getAxis(1)

    def getPitch(self):

        return self._getAxis(2)

    def getYaw(self):

        return self._getAxis(3)

    def _getAxis(self, k):

        j = self.axis_map[k]
        val = self.joystick.get_axis(abs(j))
        if abs(val) < Controller.STICK_DEADBAND:
            val = 0
        return (-1 if j<0 else +1) * val

class GameController(Controller):

    def __init__(self, axis_map, button_id):

        Controller.__init__(self, axis_map)
        self.button_id = button_id
        self.button_is_down = False
        self.switch_value = -1

    def _getAuxValue(self):

        return self.joystick.get_button(self.button_id)
        
    def getAux(self):

        if self._getAuxValue():
            if not self.button_is_down:
                self.switch_value = -self.switch_value
            self.button_is_down = True
        else:
            self.button_is_down = False
        return self.switch_value

class SpringyThrottleController(GameController):

    THROTTLE_SCALE = .01
    
    def __init__(self, axis_map, button_id):

        GameController.__init__(self, axis_map, button_id)
        
        self.throttleval = -1

    def getThrottle(self):

        self.throttleval = min(max(self.throttleval+self._getAxis(0)*SpringyThrottleController.THROTTLE_SCALE, -1), +1)

        return self.throttleval

class RcTransmitter(Controller):

    def __init__(self, axis_map, aux_id):

        Controller.__init__(self, axis_map)
        self.aux_id = aux_id
        
    def getAux(self):

        return +1 if self.joystick.get_axis(self.aux_id) > 0 else -1
        
class Xbox360(SpringyThrottleController):

    def __init__(self):

        SpringyThrottleController.__init__(self, (-1,  4, -3, 0), None)

    def _getAuxValue(self):

        return self.joystick.get_axis(2) < -.5
        

class Playstation(SpringyThrottleController):

    def __init__(self):

        SpringyThrottleController.__init__(self, (-1,  2, -3, 0), 7)

class ExtremePro3D(GameController):

    def __init__(self):

        GameController.__init__(self, (-2,  0,  1, 3), 0) # no springy throttle

class Taranis(RcTransmitter):

    def __init__(self):

        RcTransmitter.__init__(self, (0,  1,  2, 5), 3)

class Spektrum(RcTransmitter):

    def __init__(self):

        RcTransmitter.__init__(self, (1,  2,  5, 0), 4)

# Make a dictionary of controllers
controllers = {
    'Controller (Rock Candy Gamepad for Xbox 360)' : Xbox360(), 
    '2In1 USB Joystick'                            : Playstation(),
    'Wireless Controller'                          : Playstation(),    
    'Logitech Extreme 3D'                          : ExtremePro3D(),
    'FrSky Taranis Joystick'                       : Taranis(),
    'SPEKTRUM RECEIVER'                            : Spektrum()
    }

def get_controller():

    # Initialize pygame for joystick support
    pygame.display.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Find your controller
    controller_name = joystick.get_name()
    if not controller_name in controllers.keys():
        print('Unrecognized controller: %s' % controller_name)
        exit(1)
    controller = controllers[controller_name]
    controller.joystick = joystick

    return controller


if __name__ == '__main__':

    '''
    Test
    '''

    con = get_controller()
        
    while True:

        con.update()

        print('%+2.2f %+2.2f %+2.2f %+2.2f %+2.2f' %
             (con.getThrottle(), con.getRoll(), con.getPitch(), con.getYaw(), con.getAux()))
