#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('log.csv', delimiter=',')

def plot(index, label):
    plt.subplot(7, 1, index+1)
    plt.plot(data[:,2*index])
    plt.plot(data[:,2*index+1])
    plt.ylabel(label)
    plt.legend(('comp', 'ekf'))

plot(0, 'dx/dt (m/s)')
plot(1, 'dy/dt (m/s)')
plot(2, 'z (m)')
plot(3, 'phi (deg)')
plot(4, 'theta (deg)')
plot(5, 'psi (deg)')

plt.show()

