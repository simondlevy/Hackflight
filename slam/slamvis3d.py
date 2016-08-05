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


# Adapted from
# http://stackoverflow.com/questions/18853563/how-can-i-paint-the-faces-of-a-cube

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class ThreeDSlamVis(object):

    def __init__(self, map_size_cm=1000, obstacle_size_cm=10, vehicle_size_cm=25):

        fig = plt.figure(figsize=(10,10))
        self.ax = fig.gca(projection='3d')
        self.ax.set_aspect("auto")
        self.ax.set_autoscale_on(True)

        self.ax.set_xlim([-map_size_cm, +map_size_cm])
        self.ax.set_ylim([-map_size_cm, +map_size_cm])
        self.ax.set_zlim([-map_size_cm, +map_size_cm])

        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_zlabel('Z (cm)')

        self.obstacle_size_cm = obstacle_size_cm

        # Create five vertices for vehcile
        s = vehicle_size_cm
        A = [0,   0,   0]
        B = [s,   0,   0]
        C = [s,   0,   s]
        D = [0,   0,   s]
        E = [s/2, s,   s/2]

        # Make a pyramid from five faces
        pyr = [
                [A,B,C,D],
                [B,C,E],
                [C,D,E],
                [A,D,E],
                [A,B,E]
                ]

        self.ax.add_collection3d(Poly3DCollection(pyr, facecolors='red'))

    def addObstacle(self, x, y, z):

        s = self.obstacle_size_cm

        # Create eight vertices for cube
        A = [x,   y,   z]
        B = [x+s, y,   z]
        C = [x+s, y+s, z]
        D = [x,   y+s, z]
        E = [x,   y,   z+s]
        F = [x+s, y,   z+s]
        G = [x+s, y+s, z+s]
        H = [x,   y+s, z+s]

        # Make cube from six faces of four vertices each
        cube = [
                [A, B, C, D],
                [E, F, G, H],
                [F, G, C, B],
                [E, H, D, A],
                [E, F, B, A],
                [H, G, C, D],
        ]

        self.ax.add_collection3d(Poly3DCollection(cube, facecolors='white'))

    def setPose(self, x, y, z, theta):
        '''
        Sets vehicle pose: 
        X: left/right   (cm)
        Y: forward/back (cm)
        Z: up/down      (cm)
        theta: degrees
        '''
    
        return

    def redraw(self):

        # Assume no use interruption
        retval = True

        # Redraw current objects without blocking
        plt.draw()

        # Refresh display, setting flag on window close or keyboard interrupt
        try:
            plt.pause(.1)
        except:
            retval = False

        return retval


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

        slamvis.addObstacle(ox,oy,z)

        if not slamvis.redraw():
            break

        sleep(.05)

        theta = (theta + 10) % 360

        z += 2 * zdir

        if z > 500:
            zdir = -1
        if z < 10:
            zdir = +1


