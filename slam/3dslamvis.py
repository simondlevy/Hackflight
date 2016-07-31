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

from visual import box, display, cone, color, vector, materials
from math import radians

class ThreeDSlamVis(object):

    def __init__(self, display_size_pixels=1000, map_size_cm=1000, obstacle_size_cm=10):

        display(width=display_size_pixels,height=display_size_pixels,title='3D SLAM')

        # Floor for reference
        box(pos = (0, map_size_cm*FLOOR_Y, map_size_cm*FLOOR_Z), 
                   length=map_size_cm, height=1, width=map_size_cm, 
                   material=materials.rough)

        self.vehicle = cone(pos = (0, map_size_cm*FLOOR_Y+CONE_YOFFSET, map_size_cm*FLOOR_Z), 
                axis=(CONE_LENGTH,0,0), radius=CONE_RADIUS, color=color.red)

        self.obstacle_size_cm = obstacle_size_cm

        # Store vehicle heading angle for rotation by setPose()
        self.theta = 0

    def addObstacle(self, x, y, z):

        box(pos = (x, z,y), 
            length=self.obstacle_size_cm, height=self.obstacle_size_cm, width=self.obstacle_size_cm, 
            material=materials.diffuse)


    def setPose(self, x, y, z, theta):
        '''
        Sets vehicle pose: 
        X: left/right   (cm)
        Y: forward/back (cm)
        Z: up/down      (cm)
        theta: degrees
        '''

        # Rotate vehicle by difference between current angle and desired angle
        self.vehicle.rotate(angle=radians(self.theta - theta), axis=(0,1,0))

        # VPython uses Y for up/down; we use Z
        self.vehicle.pos = vector(x, z, y)

        self.theta = theta

if __name__ == '__main__':

    from random import uniform
    from time import sleep

    slamvis = ThreeDSlamVis()

    x,y,z,theta = 0,0,0,0
    zdir = +1

    while True:

        slamvis.setPose(x,y,z,theta)

        ox = int(uniform(-500,500))
        oy = int(uniform(-500,500))
        oz = int(uniform(10,500))

        slamvis.addObstacle(ox,oy,oz)

        sleep(.05)

        theta = (theta + 10) % 360

        z += 2 * zdir

        if z > 500:
            zdir = -1
        if z < 10:
            zdir = +1

