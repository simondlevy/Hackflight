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

FLOOR_X      =   0
FLOOR_Y      = -30
FLOOR_Z      =  20
FLOOR_S      = 100

CONE_LENGTH  =   3
CONE_YOFFSET =   2
CONE_RADIUS  =   1

from visual import box, display, cone, color, rate, vector, materials
from time import sleep
from math import pi, radians

class ThreeDSlamVis(object):

    def __init__(self, size_pixels=800, size_cm=10000):

        display(width=size_pixels,height=size_pixels,title='3D SLAM')

        floor = box(pos = (FLOOR_X, FLOOR_Y, FLOOR_Z), 
                length=FLOOR_S, height=1, width=FLOOR_S, 
                material=materials.rough)

        self.vehicle = cone(pos = (FLOOR_X, FLOOR_Y+CONE_YOFFSET, FLOOR_Z), 
                axis=(CONE_LENGTH,0,0), radius=CONE_RADIUS, color=color.red)

        self.size_pixels = size_pixels
        self.size_cm = size_cm

        self.pose = 0,0,0,0

    def addObstacle(self, x, y, z):

        return

    def setPose(self, x, y, z, theta):

        self.vehicle.rotate(angle=radians(self.pose[3] - theta), axis=(0,1,0))

        self.vehicle.pos += vector(0,1,0)

        print(self.vehicle.pos[1])

        self.pose = x, y, z, theta

if __name__ == '__main__':

    slamvis = ThreeDSlamVis()

    theta = 0

    while True:

        slamvis.setPose(0,0,0,theta)

        sleep(.1)

        #theta = (theta + 1) % 360
