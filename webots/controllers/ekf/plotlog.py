#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

time = data[:,0]

def plot2(j, k1, k2, label, lo, hi):
    plt.subplot(7, 1, j)
    plt.plot(time, data[:, k1])
    plt.plot(time, data[:, k2])
    plt.ylim((lo, hi))
    plt.ylabel(label)
    plt.legend(('true', 'ekf'))

def plot3(j, k1, k2, k3, label, lo, hi):
    plt.subplot(7, 1, j)
    plt.plot(time, data[:, k1])
    plt.plot(time, data[:, k2])
    plt.plot(time, data[:, k3])
    plt.ylim((lo, hi))
    plt.ylabel(label)
    plt.legend(('true', 'comp', 'ekf'))

VEL_LIM = 1.5
Z_LIM = 1
ANG_LIM = 10

plot3(1, 1, 2, 3, 'dx/dt (m/s)', -VEL_LIM, +VEL_LIM)
plot3(2, 4, 5, 6, 'dy/dt (m/s)', -VEL_LIM, +VEL_LIM)
plot3(3, 7, 8, 9, 'z (m)', 0, Z_LIM);
plot3(4, 10, 11, 12, 'dz/dt (m/s)', -VEL_LIM, +VEL_LIM)
plot2(5, 13, 14, 'phi (deg)', -ANG_LIM, +ANG_LIM)
plot2(6, 15, 16, 'theta (deg)', -ANG_LIM, +ANG_LIM)
plot2(7, 17, 18, 'psi (deg)', -180, +180)

plt.xlabel('time (s)')

plt.show()

