#!/usr/bin/env python
'''
3dslamvis.py : simple 3D SLAM visualization in Python

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

# proportional 
FLOOR_Y      = -.3
FLOOR_Z      =  .2

# centimeters
CONE_LENGTH  =   30
CONE_YOFFSET =   20
CONE_RADIUS  =   10

from visual import box, display, cone, color, rate, vector, materials
from time import sleep
from math import pi, radians

class ThreeDSlamVis(object):

    def __init__(self, display_size_pixels=1000, map_size_cm=1000):

        display(width=display_size_pixels,height=display_size_pixels,title='3D SLAM')

        floor = box(pos = (0, map_size_cm*FLOOR_Y, map_size_cm*FLOOR_Z), 
                length=map_size_cm, height=1, width=map_size_cm, 
                material=materials.rough)

        self.vehicle = cone(pos = (0, map_size_cm*FLOOR_Y+CONE_YOFFSET, map_size_cm*FLOOR_Z), 
                axis=(CONE_LENGTH,0,0), radius=CONE_RADIUS, color=color.red)

        self.pose = 0,0,0,0

    def addObstacle(self, x, y, z):

        return

    def setPose(self, x, y, z, theta):
        '''
        Sets vehicle pose (X,Y,Z coordinates in cm and heading theta in degrees).
        '''

        self.vehicle.rotate(angle=radians(self.pose[3] - theta), axis=(0,1,0))

        # VPython uses Y for up/down; we use Z
        self.vehicle.pos = vector(x, z, y)

        self.pose = x, y, z, theta

if __name__ == '__main__':

    slamvis = ThreeDSlamVis()

    x,y,z,theta = 0,0,0,0

    while True:

        slamvis.setPose(x,y,z,theta)

        sleep(.1)

        theta = (theta + 10) % 360
        x += 10
        y += 20
        x += 10

