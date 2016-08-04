#!/usr/bin/env python

# Adapted from
# http://stackoverflow.com/questions/18853563/how-can-i-paint-the-faces-of-a-cube

import matplotlib.pyplot as plt
import numpy as np
from itertools import product, combinations
from matplotlib.patches import Rectangle
import mpl_toolkits.mplot3d.art3d as art3d

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_aspect("auto")
ax.set_autoscale_on(True)


r = [-10, 10]
for s, e in combinations(np.array(list(product(r,r,r))), 2):
    if np.sum(np.abs(s-e)) == r[1]-r[0]:
        ax.plot3D(*zip(s,e), color="b")

colors = ['b', 'g', 'r', 'c', 'm', 'y']
for i, (z, zdir) in enumerate(product([-2,2], ['x','y','z'])):
    side = Rectangle((-2, -2), 4, 4, facecolor=colors[i])
    ax.add_patch(side)
    art3d.pathpatch_2d_to_3d(side, z=z, zdir=zdir)


plt.show()
