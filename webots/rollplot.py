#!/usr/bin/python3

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

    plt.subplot(2, 1, 1)
    plt.plot(phi)

    plt.subplot(2, 1, 2)
    plt.plot(dphi)

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
