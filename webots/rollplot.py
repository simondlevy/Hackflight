#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt


def main():

    phi = []
    dphi = []

    while True:

        words = []

        try:
            words = input().split()

        except:
            break

        if len(words) == 2:

            phi.append(float(words[0]))
            dphi.append(float(words[1]))

    n = len(phi)
    time = np.linspace(0, n/32, n)

    plt.subplot(2, 1, 1)
    plt.plot(time, phi)
    plt.xticks([], [])
    plt.ylabel('Angle (deg)')

    plt.subplot(2, 1, 2)
    plt.plot(time, dphi)
    plt.xlabel('Time (sec)')
    plt.ylabel('Angular velocity (deg/sec)')

    plt.show()

    '''
    plt.subplot(2, 1, 1)
    plt.plot(time, throttle)
    plt.xticks([], [])
    plt.ylabel('throttle')

    plt.subplot(2, 1, 2)
    plt.plot(time, dz_target)
    plt.xlabel('time (s)')
    plt.ylabel('dz target (m/s)')
    '''

    plt.show()


main()
