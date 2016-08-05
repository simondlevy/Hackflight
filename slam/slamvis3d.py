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

    def addObstacle(self, x, y, z):

        s = self.obstacle_size_cm

        A = [x,   y,   z]
        B = [x+s, y,   z]
        C = [x+s, y+s, z]
        D = [x,   y+s, z]
        E = [x,   y,   z+s]
        F = [x+s, y,   z+s]
        G = [x+s, y+s, z+s]
        H = [x,   y+s, z+s]

        cube = [
                [A, B, C, D],
                [E, F, G, H],
                [F, G, C, B],
                [E, H, D, A],
                [E, F, B, A],
                [H, G, C, D],
        ]

        self.ax.add_collection3d(Poly3DCollection(cube))

if __name__ == '__main__':

    slam = ThreeDSlamVis()

    plt.show(block=False)

    x,y,z = 0,0,0

    while True:

        if x < 600:
            slam.addObstacle(x,y,z)
        plt.draw()
        try:
            plt.pause(1)
        except:
            break

        x += 300


