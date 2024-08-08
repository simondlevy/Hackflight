#!/usr/bin/python3

import matplotlib.pyplot as plt


def main():

    time = []
    throttle = []
    dz_target = []

    while True:

        words = []

        try:
            words = input().split()

        except:
            break

        if len(words) == 3:

            time.append(float(words[0]))
            throttle.append(float(words[1]))
            dz_target.append(float(words[2]))

    plt.subplot(2, 1, 1)
    plt.plot(time, throttle)
    plt.xticks([], [])
    plt.ylabel('throttle')

    plt.subplot(2, 1, 2)
    plt.plot(time, dz_target)
    plt.xlabel('time (s)')
    plt.ylabel('dz target (m/s)')

    plt.show()


main()
