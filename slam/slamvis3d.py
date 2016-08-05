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

def _translate(pt, x, y, z):

    return pt[0]+x, pt[1]+y, pt[2]+z

def _rotate(pt, theta_degrees):

    return pt[0], pt[1], pt[2]

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

        # White axes on black background
        self.ax.set_axis_bgcolor('black')
        self.ax.tick_params(axis='x', colors='white')
        self.ax.tick_params(axis='y', colors='white')
        self.ax.tick_params(axis='z', colors='white')
        self.ax.xaxis.label.set_color('white')
        self.ax.yaxis.label.set_color('white')
        self.ax.zaxis.label.set_color('white')
        self.ax.grid(False)
        self.ax.w_xaxis.set_pane_color((0, 0, 0, 0))
        self.ax.w_yaxis.set_pane_color((0, 0, 0, 0))
        self.ax.w_zaxis.set_pane_color((0, 0, 0, 0))

        self.obstacle_size_cm = obstacle_size_cm
        self.vehicle_size_cm = vehicle_size_cm

        self._add_vehicle(0,0,0,0)

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

        self.ax.collections.remove(self.vehicle)

        self._add_vehicle(x, y, z, theta)
    
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

    def _add_vehicle(self, x, y, z, theta):

        # Convert vehicle size to length, width, height
        l = self.vehicle_size_cm
        w = l / 2
        h = l / 3

        # Create five vertices for vehcile, centered at origin
        s = self.vehicle_size_cm
        A = (0,   0,  0)
        B = (w,   0,  0)
        C = (w,   0,  h)
        D = (0,   0,  h)
        E = (w/2, l,  h/2)

        # Rotate the vertices by the yaw (heading) angle theta
        A = _rotate(A, theta)
        B = _rotate(B, theta)
        C = _rotate(C, theta)
        D = _rotate(D, theta)
        E = _rotate(E, theta)

        # Add the x,y,z offset to the vertices
        A = _translate(A, x, y, z)
        B = _translate(B, x, y, z)
        C = _translate(C, x, y, z)
        D = _translate(D, x, y, z)
        E = _translate(E, x, y, z)

        # Make a pyramid from five faces built from vertices
        pyr = (
                (A,B,C,D),
                (B,C,E),
                (C,D,E),
                (A,D,E),
                (A,B,E)
                )

        self.vehicle = Poly3DCollection(pyr, facecolors='red')

        self.ax.add_collection3d(self.vehicle)


if __name__ == '__main__':

    from random import uniform
    from time import sleep

    mapsize_cm = 300

    slamvis = ThreeDSlamVis(map_size_cm=mapsize_cm)

    x,y,z,theta = 0,0,0,0
    zdir = +1

    while True:

        slamvis.setPose(x,y,z,theta)

        ox = int(uniform(-mapsize_cm,mapsize_cm))
        oy = int(uniform(-mapsize_cm,mapsize_cm))

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


