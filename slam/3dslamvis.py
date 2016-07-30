#!/usr/bin/env python

from visual import box, display, cone, color, rate, vector
from time import sleep
from math import pi

class ThreeDSlamVis(object):

    def __init__(self, display_size=800):

        display(width=display_size,height=display_size,title='3D SLAM')

        X = 0
        Y = -30
        Z = 20

        self.floor = box(pos = (X, Y, Z), length=100, height=1, width=100)

        self.vehicle = cone(pos = (X,Y+10, Z), axis=(3,0,0), radius=1, color=color.red)
      
    def addObstacle(self, x, y, z):

        return

    def setPose(self, x, y, z, theta):

        return

if __name__ == '__main__':

    slamvis = ThreeDSlamVis()

    while True:

        slamvis.vehicle.rotate(angle=pi/4, axis=(0,1,0))

        sleep(.5)

