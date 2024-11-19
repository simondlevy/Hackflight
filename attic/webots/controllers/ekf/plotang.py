#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

time = data[:,0]

def plot(j, k, label, lo, hi):
    plt.subplot(3, 1, j)
    plt.plot(time, data[:, k])
    plt.plot(time, data[:, k+1])
    plt.ylim((lo, hi))
    plt.ylabel(label)
    plt.legend(('true', 'ekf'))

ANG_LIM = 10

plot(1, 13, 'phi (deg)', -ANG_LIM, +ANG_LIM)
plot(2, 15, 'theta (deg)', -ANG_LIM, +ANG_LIM)
plot(3, 17, 'psi (deg)', -180, +180)

plt.xlabel('time (s)')

plt.show()

