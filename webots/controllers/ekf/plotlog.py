#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

def plot(index, label, lo, hi):
    time = data[:,0]
    plt.subplot(7, 1, index+1)
    plt.plot(time, data[:, 2*index+1])
    plt.plot(time, data[:, 2*index+2])
    plt.ylim((lo, hi))
    plt.ylabel(label)
    plt.legend(('comp', 'ekf'))

VEL_LIM = 2
Z_LIM = 2
ANG_LIM = 10

plot(0, 'dx/dt (m/s)', -VEL_LIM, +VEL_LIM)
plot(1, 'dy/dt (m/s)', -VEL_LIM, +VEL_LIM)
plot(2, 'z (m)', 0, Z_LIM)
plot(3, 'dz/dt (m/s)', -VEL_LIM, +VEL_LIM)
plot(4, 'phi (deg)', -ANG_LIM, +ANG_LIM)
plot(5, 'theta (deg)', -ANG_LIM, +ANG_LIM)
plot(6, 'psi (deg)', -180, +180)

plt.show()

