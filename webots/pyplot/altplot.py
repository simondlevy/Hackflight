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

    plt.subplot(4, 1, 1)
    plt.plot(time, setpoint)
    plt.plot(time, z)
    plt.xticks([], [])
    plt.ylabel('Altitude (m)')
    plt.legend(['Target', 'Actual'])

    '''
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
    '''

    plt.show()


main()
