#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('tmp.csv', delimiter=',', skiprows=3)

plt.subplot(3, 1, 1)
plt.plot(data[:,0])
plt.legend(['throttle'])

plt.subplot(3, 1, 2)
plt.plot(data[:,1])
plt.plot(data[:,2])
plt.legend(['roll', 'pitch'])

plt.subplot(3, 1, 3)
plt.plot(data[:,3])
plt.legend(['yaw'])

plt.show()
