#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt


def plot(data, idx, label, lim):

    plt.subplot(6, 1, idx+1)
    plt.plot(data[:, idx])
    plt.plot(data[:, idx+6])
    plt.ylabel(label)
    plt.ylim((-lim, +lim))
    plt.legend(('Madg', 'EKF'))


def main():

    data = np.loadtxt('log.csv', delimiter=',')

    plot(data, 0, 'phi', 90)
    plot(data, 1, 'dphi', 400)
    plot(data, 2, 'theta', 90)
    plot(data, 3, 'dtheta', 400)
    plot(data, 4, 'psi', 180)
    plot(data, 5, 'dpsi', 400)

    plt.show()


main()
