#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

plt.subplot(2, 1, 1)
plt.plot(data[:,0])
plt.plot(data[:,1])
plt.ylabel('z (m)')
plt.legend(('comp', 'ekf'))

plt.subplot(2, 1, 2)
plt.plot(data[:,2])
plt.plot(data[:,3])
plt.ylabel('dz/dt (m/s)')
plt.legend(('comp', 'ekf'))

plt.show()

