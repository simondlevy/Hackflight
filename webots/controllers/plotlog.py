#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

plt.subplot(2, 1, 1)

plt.plot(data[:, 0])

plt.ylabel('yaw demand')

plt.subplot(2, 1, 2)

plt.plot(data[:,1], '+')
plt.plot(data[:,2], '*')
plt.plot(data[:,3])
plt.plot(data[:,4])

plt.ylim([54.5, 56])

plt.legend(['m1', 'm2', 'm3', 'm4'])

plt.show()

