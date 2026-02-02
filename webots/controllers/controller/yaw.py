#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('tmp.csv', delimiter=',', skiprows=3)

SCALE = -1.5e4

plt.plot(data[:,0])
plt.plot(SCALE * data[:,1])
plt.legend(('old', 'new'))

plt.show()
