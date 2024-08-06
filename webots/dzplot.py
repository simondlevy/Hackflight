#!/usr/bin/python3

import sys
import numpy as np
import matplotlib.pyplot as plt


def main():

    times = []
    ztarget = []
    z = []
    dz = []
    motor = []
    fitness = 0

    while True:

        words = []

        try:
            line = input()
            words = input().split()

        except:
            break

        if len(words) == 5:

            times.append(float(words[0]))
            ztarget.append(float(words[1]))
            z.append(float(words[2]))
            dz.append(float(words[3]))
            motor.append(float(words[4]))

    plt.subplot(3,1,1)
    plt.plot(times, z)
    plt.plot(times, ztarget, 'r')
    plt.legend(['actual', 'target'])
    plt.xticks([], [])
    plt.ylabel('z (m)')

    plt.subplot(3,1,2)
    plt.plot(times, dz)
    plt.xticks([], [])
    plt.ylabel('dz/dt (m/s)')

    plt.subplot(3,1,3)
    plt.plot(times, motor)
    plt.xlabel('time (s)')
    plt.ylabel('motor (rad/s)')

    plt.show()

main()
