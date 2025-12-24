#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

gt = np.loadtxt('groundtruth.csv')
ss = np.loadtxt('simsens.csv')

plt.plot(gt)
plt.plot(ss)
plt.xlabel('timestep')
plt.ylabel('distance (mm)')
plt.legend(('groundtruth', 'simsens'))
plt.show()

