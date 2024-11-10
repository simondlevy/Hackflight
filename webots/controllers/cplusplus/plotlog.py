#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

plt.subplot(2, 1, 1)
plt.plot(data[:,0], '-x')
plt.plot(data[:,1], '-+')
plt.legend(('est', 'true'))
plt.ylabel('z (m)')

plt.subplot(2, 1, 2)
plt.plot(data[:,2], '-x')
plt.plot(data[:,3], '-+')
plt.ylabel('dz/dt (m/s)')

plt.show()

