#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt


def main():

    data = np.loadtxt('controllers/cplusplus/altitude.csv',
            delimiter=',', skiprows=1)

    time = data[:, 0]
    setpoint = data[:, 1]
    z = data[:, 2]
    dz = data[:, 3]
    output = data[:, 4]

    plt.subplot(3, 1, 1)
    plt.plot(time, setpoint)
    plt.plot(time, z)
    plt.xticks([], [])
    plt.ylabel('Altitude (m)')
    plt.legend(['Target', 'Actual'])

    plt.subplot(3, 1, 2)
    plt.plot(time, dz)
    plt.xticks([], [])
    plt.ylabel('Climb rate (m/s)')

    plt.subplot(3, 1, 3)
    plt.plot(time, output)
    plt.ylabel('Thrust')

    plt.show()


main()
