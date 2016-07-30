#!/usr/bin/env python

from visual import box, display, cone, color

class ThreeDSlamVis(object):

    def __init__(self, display_size=800):

        display(width=display_size,height=display_size,title='3D SLAM')

        X = 0
        Y = -30
        Z = 20

        box(pos = (X, Y, Z), length=100, height=1, width=100)

        self.vehicle = cone(pos = (X,Y+10, Z), axis=(3,0,0), radius=1, color=color.red)
      
    def addObstacle(self, x, y, z):

        return

    def setPose(self, x, y, z, theta):

        return

if __name__ == '__main__':

    slamvis = ThreeDSlamVis()

