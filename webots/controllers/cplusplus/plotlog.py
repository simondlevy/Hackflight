#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

plt.subplot(4, 1, 1)
plt.plot(data[:,0])
plt.plot(data[:,1])
plt.ylabel('dx/dt (m/s)')
plt.legend(('comp', 'ekf'))

plt.subplot(4, 1, 2)
plt.plot(data[:,2])
plt.plot(data[:,3])
plt.ylabel('z (m)')
plt.legend(('comp', 'ekf'))

plt.subplot(4, 1, 3)
plt.plot(data[:,4])
plt.plot(data[:,5])
plt.ylabel('dz/dt (m/s)')
plt.legend(('comp', 'ekf'))

plt.subplot(4, 1, 4)
plt.plot(data[:,6])
plt.plot(data[:,7])
plt.ylabel('theta (deg)')
plt.legend(('true', 'ekf'))

plt.show()

