#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt


def main():

    demand = []
    phi = []
    dphi = []

    while True:

        words = []

        try:
            words = input().split()

        except:
            break

        if len(words) == 3:

            demand.append(float(words[0]))
            phi.append(float(words[1]))
            dphi.append(float(words[2]))

    n = len(phi)
    time = np.linspace(0, n/32, n)

    lotime = 4

    plt.subplot(3, 1, 1)
    plt.plot(time, demand)
    plt.xlim([lotime, time[-1]])
    plt.xticks([], [])
    plt.ylabel('Demand (deg)')

    plt.subplot(3, 1, 2)
    plt.plot(time, phi)
    plt.xticks([], [])
    plt.ylabel('Angle (deg)')
    plt.xlim([lotime, time[-1]])

    plt.subplot(3, 1, 3)
    plt.plot(time, dphi)
    plt.xlabel('Time (sec)')
    plt.ylabel('Angular velocity (deg/sec)')
    plt.xlim([lotime, time[-1]])

    plt.show()


main()
