#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

time = data[:,0]

plt.subplot(3, 1, 1)

plt.plot(time, data[:,2], '+')
plt.plot(time, data[:,3], '*')
plt.plot(time, data[:,4])
plt.plot(time, data[:,5])

mid = 55.375
mar = 0.6

plt.plot(time, mid * np.ones(time.shape))

plt.ylim([mid-mar, mid+mar])

plt.legend(['m1', 'm2', 'm3', 'm4'])

plt.ylabel('motors (rad / sec)')

plt.subplot(3, 1, 2)
plt.plot(time, data[:,6])
plt.ylabel('dpsi/dt (deg / sec)')

plt.subplot(3, 1, 3)
plt.plot(time, data[:,7])
plt.ylabel('psi (deg)')

plt.xlabel('time (sec)')
plt.show()

