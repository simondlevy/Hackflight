#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

time = data[:,0]

plt.subplot(2, 1, 1)
plt.ylabel('accel z (g)')
plt.plot(time, data[:,1]) 

plt.subplot(2, 1, 2)
plt.ylabel('dz/dt (m/s)')
plt.plot(time, data[:,2]) 

plt.xlabel('time (sec)')

plt.show()

