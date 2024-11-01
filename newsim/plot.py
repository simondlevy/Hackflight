#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

time = data[:,0]
motor = data[:,1]
z = data[:,2]
dz = data[:,3]

plt.subplot(3, 1, 1)
plt.plot(time, motor)
plt.xticks([], [])
plt.ylabel('motor (rad/s) ')

plt.subplot(3, 1, 2)
plt.plot(time, dz)
plt.xticks([], [])
plt.ylabel('climb rate (m/s) ')

plt.subplot(3, 1, 3)
plt.plot(time, z)
plt.xlabel('time (s) ')
plt.ylabel('altitude (m) ')

plt.show()
