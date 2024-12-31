#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

def plot(data, idx, label, lim):
    plt.subplot(5, 1, idx+1)
    plt.plot(data[:, idx*2])
    plt.plot(data[:, idx*2+1])
    plt.ylabel(label)
    plt.ylim((-lim, +lim))
    plt.legend(('usfs', 'mpu6050'))

def plot_angle(data, idx, label):
    plot(data, idx, label, 90)

def plot_rate(data, idx, label):
    plot(data, idx, label, 400)

data = np.genfromtxt('log.csv', delimiter=',')

plot_angle(data, 0, 'phi')
plot_angle(data, 1, 'theta')
plot_rate(data, 2, 'dphi')
plot_rate(data, 3, 'dtheta')
plot_rate(data, 4, 'dpsi')

plt.show()
