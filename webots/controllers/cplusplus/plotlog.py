#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

plt.subplot(4, 1, 1)
plt.plot(data[:,0])
plt.ylabel('raw flow')

plt.subplot(4, 1, 2)
plt.plot(data[:,1])
plt.ylabel('rangefinder (m)')

plt.subplot(4, 1, 3)
plt.plot(data[:,2])
plt.ylabel('gyro (m/s)')

plt.subplot(4, 1, 4)
plt.plot(data[:,3], '-x')
plt.plot(data[:,4])
plt.ylabel('velocity (m/s)')
plt.legend(('true', 'est'))

plt.show()

