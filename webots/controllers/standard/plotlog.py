#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from sys import argv


def main():

    data = np.loadtxt(argv[1], delimiter=',')

    hover_rpm = float(argv[2]) * np.ones(data.shape[0])

    plt.subplot(3, 1, 1)
    plt.plot(data[:,0])
    plt.plot(hover_rpm)
    plt.ylabel('RPM')

    plt.subplot(3, 1, 2)
    plt.plot(data[:,1])
    plt.ylabel('U1')

    plt.subplot(3, 1, 3)
    plt.plot(data[:,2])
    plt.ylabel('dZ/dt')

    plt.show()



main()
