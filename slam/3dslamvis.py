#!/usr/bin/env python

from visual import box, display, cone, color

class ThreeDSlamVis(object):

    def __init__(self, display_size=800):

        display(width=display_size,height=display_size,title='3D SLAM')

        box(pos=(0,-30,20), length=80, height=0.2, width=80)

        cone(pos=(0,-1, 2), axis=(3,0,0), radius=1, color=color.red)
      
    def addObstacle(self, x, y, z):

        return

    def setPose(self, x, y, z, theta):

        return

if __name__ == '__main__':

    slamvis = ThreeDSlamVis()

