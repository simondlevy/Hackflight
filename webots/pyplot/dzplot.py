#!/usr/bin/python3

import matplotlib.pyplot as plt


def main():

    demand = []
    dz = []
    output = []

    while True:

        words = []

        try:
            words = input().split()

        except:
            break

        if len(words) == 3:

            demand.append(float(words[0]))
            dz.append(float(words[1]))
            output.append(float(words[2]))

    plt.subplot(2, 1, 1)
    plt.plot(demand, 'r')
    plt.plot(dz)
    plt.legend(['demand', 'dz'])

    plt.subplot(2, 1, 2)
    plt.plot(output)

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
