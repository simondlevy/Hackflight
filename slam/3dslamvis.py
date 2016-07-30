#!/usr/bin/env python

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

    def __init__(self, display_size=800):

        display(width=display_size,height=display_size,title='3D SLAM')

        floor = box(pos = (FLOOR_X, FLOOR_Y, FLOOR_Z), 
                length=FLOOR_S, height=1, width=FLOOR_S, 
                material=materials.rough)

        self.vehicle = cone(pos = (FLOOR_X, FLOOR_Y+CONE_YOFFSET, FLOOR_Z), 
                axis=(CONE_LENGTH,0,0), radius=CONE_RADIUS, color=color.red)

        self.pose = 0,0,0,0

    def addObstacle(self, x, y, z):

        return

    def setPose(self, x, y, z, theta):

        self.vehicle.rotate(angle=radians(self.pose[3] - theta), axis=(0,1,0))

        self.pose = x, y, z, theta

if __name__ == '__main__':

    slamvis = ThreeDSlamVis()

    theta = 0

    while True:

        slamvis.setPose(0,0,0,theta)

        sleep(.01)

        theta = (theta + 1) % 360
