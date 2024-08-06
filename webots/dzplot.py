#!/usr/bin/python3

import sys
import numpy as np
import matplotlib.pyplot as plt

PLOT_AFTER_SECONDS = 6

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

        if times[-1] > PLOT_AFTER_SECONDS:
            break

    plt.subplot(2,1,1)
    plt.plot(times, dz)
    plt.xticks([], [])
    plt.ylabel('dz/dt (m/s)')

    plt.subplot(2,1,2)
    plt.plot(times, thrust)
    plt.xlabel('time (s)')
    plt.ylabel('thrust (rad/s)')

    plt.show()

main()
