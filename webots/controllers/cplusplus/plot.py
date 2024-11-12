#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

time = data[:,0]

plt.subplot(2, 1, 1)
plt.plot(time, data[:,1])
plt.ylabel('motor (rad/s)')

plt.subplot(2, 1, 2)
plt.plot(time, data[:,2])
plt.ylabel('altitude (m)')

plt.xlabel('time (s)')

plt.show()

