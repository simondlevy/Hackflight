#!/usr/bin/env python

from visual import box, display

class ThreeDSlamVis(object):

    def __init__(self, display_size=800):

        display(width=display_size,height=display_size,title='3D SLAM')

        box(pos=(0,-3,2), length=8, height=0.2, width=8)

      
    def addObstacle(self, x, y, z):

        return

    def setPose(self, x, y, z, theta):

        return

if __name__ == '__main__':

    slamvis = ThreeDSlamVis()

