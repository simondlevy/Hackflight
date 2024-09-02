#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt


def main():

    data = np.loadtxt('controllers/cplusplus/roll.csv',
            delimiter=',', skiprows=1)

    time = data[:, 0]
    roll_stick = data[:, 1]
    dy = data[:, 2]
    angle_demand = data[:, 3]
    phi = data[:, 4]

    plt.subplot(2, 1, 1)
    plt.plot(time, roll_stick)
    plt.plot(time, dy)
    plt.ylim((-1.1,1.1))
    plt.ylabel('Velocity (m/s)')
    plt.legend(('Setpoint', 'Actual'))
    plt.xticks([], [])

    plt.subplot(2, 1, 2)
    plt.plot(time, angle_demand)
    plt.plot(time, phi)
    plt.ylim((-15,15))
    plt.ylabel('Angle (deg)')
    plt.legend(['Target', 'Actual'])

    plt.show()


main()
