#!/usr/bin/python3

import sys
import numpy as np
import matplotlib.pyplot as plt


def main():

    times = []
    thrust = []
    dz = []
    fitness = 0

    while True:

        line = input()

        words = line.split()

        times.append(float(words[0]))

        dz.append(float(words[1]))

        thrust.append(float(words[2]))

        if len(times) > 300:
            break

    plt.subplot(2,1,1)
    plt.plot(times, dz)

    plt.subplot(2,1,2)
    plt.plot(times, thrust)

    plt.show()

main()
