#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt


def main():

    data = np.loadtxt('controllers/cplusplus/roll.csv',
            delimiter=',', skiprows=1)

    time = data[:, 0]
    setpoint = data[:, 1]
    dy = data[:, 2]
    phi = data[:, 3]
    dphi = data[:, 4]
    output = data[:, 5]

    plt.subplot(4, 1, 1)
    plt.plot(time, setpoint)
    plt.plot(time, dy)
    plt.xticks([], [])
    plt.ylabel('Velocity (m/s)')
    plt.legend(['Target', 'Actual'])

    plt.subplot(4, 1, 2)
    plt.plot(time, phi)
    plt.xticks([], [])
    plt.ylabel('Angle (deg)')

    plt.subplot(4, 1, 3)
    plt.plot(time, dphi)
    plt.xticks([], [])
    plt.ylabel('Ang. vel. (deg/s)')

    plt.subplot(4, 1, 4)
    plt.plot(time, output)
    plt.xlabel('Time (sec)')
    plt.ylabel('Output')

    plt.show()


main()
