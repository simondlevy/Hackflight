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
import numpy as np
from itertools import product, combinations
from matplotlib.patches import Rectangle
import mpl_toolkits.mplot3d.art3d as art3d
from time import sleep

class ThreeDSlamVis(object):

    def __init__(self, map_size_cm=1000, obstacle_size_cm=100):

        fig = plt.figure(figsize=(10,10))
        self.ax = fig.gca(projection='3d')
        self.ax.set_aspect("auto")
        self.ax.set_autoscale_on(True)

        r = [-map_size_cm, map_size_cm]
        for s, e in combinations(np.array(list(product(r,r,r))), 2):
            if np.sum(np.abs(s-e)) == r[1]-r[0]:
                self.ax.plot3D(*zip(s,e), color="b")

        self.obstacle_size_cm = obstacle_size_cm

        fig.canvas.mpl_connect('close_event', self._handle_close)
        self.is_open = True

    def _handle_close(self):

        print('close')

        self.is_open = False

    def addObstacle(self, x, y, z):

        if not self.is_open:
            return

        colors = ['b', 'g', 'r', 'c', 'm', 'y']

        s = self.obstacle_size_cm

        for i, (z, zdir) in enumerate(product([-s,s], ['x','y','z'])):
            side = Rectangle((x-s, y-s), 2*s, 2*s, facecolor=colors[i])
            self.ax.add_patch(side)
            art3d.pathpatch_2d_to_3d(side, z=z, zdir=zdir)

if __name__ == '__main__':

    slam = ThreeDSlamVis()

    plt.show(block=False)

    x,y,z = 0,0,0

    while slam.is_open:

        slam.addObstacle(x,y,z)
        plt.draw()
        try:
            plt.pause(1)
        except:
            break

        x += 300


