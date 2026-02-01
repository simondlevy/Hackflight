#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('tmp.csv', delimiter=',', skiprows=3)

plt.subplot(4, 1, 1)
plt.plot(data[:,0])

plt.subplot(4, 1, 2)
plt.plot(data[:,1])

plt.subplot(4, 1, 3)
plt.plot(data[:,2])

plt.subplot(4, 1, 4)
plt.plot(data[:,3])

plt.show()
