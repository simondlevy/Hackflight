#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

time = data[:,0]

def plot(j, k, label, lo, hi):
    plt.subplot(2, 1, j)
    plt.plot(time, data[:, k], '-o')
    plt.plot(time, data[:, k+1], '-x')
    plt.plot(time, data[:, k+2])
    plt.ylim((lo, hi))
    plt.ylabel(label)
    plt.legend(('true', 'comp', 'ekf'))

VEL_LIM = 1.5

plot(1, 1, 'dx/dt (m/s)', -VEL_LIM, +VEL_LIM)
plot(2, 4, 'dy/dt (m/s)', -VEL_LIM, +VEL_LIM)

plt.xlabel('time (s)')

plt.show()

