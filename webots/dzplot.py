#!/usr/bin/python3

import sys
import numpy as np
import matplotlib.pyplot as plt


def main():

    time = []
    ztarget = []
    z = []
    dztarget = []
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

        if len(words) == 6:

            time.append(float(words[0]))
            ztarget.append(float(words[1]))
            z.append(float(words[2]))
            dztarget.append(float(words[3]))
            dz.append(float(words[4]))
            motor.append(float(words[5]))

    plt.subplot(3,1,1)
    plt.plot(time, z)
    plt.plot(time, ztarget, 'r')
    plt.legend(['actual', 'target'])
    plt.xticks([], [])
    plt.ylabel('z (m)')

    plt.subplot(3,1,2)
    plt.plot(time, dz)
    plt.plot(time, dztarget, 'r')
    plt.legend(['actual', 'target'])
    plt.xticks([], [])
    plt.ylabel('dz/dt (m/s)')

    plt.subplot(3,1,3)
    plt.plot(time, motor)
    plt.xlabel('time (s)')
    plt.ylabel('motor (rad/s)')

    plt.show()

main()
