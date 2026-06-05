#!/usr/bin/python3
'''
Copyright (C) 2026 Simon D. Levy

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, in version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
'''

import numpy as np
import matplotlib.pyplot as plt

FILENAME = 'log.csv'
ZMAX = 1.5
VELMAX = 2.0
Z_TAKEOFF = 0.05
MARGIN = 20


def plotmid(time, value):
    plt.plot(time, value * np.ones(len(time)), 'k-')


def main():

    try:
        data = np.loadtxt(FILENAME, delimiter=',', skiprows=1)
    except FileNotFoundError:
        print('File %s not found' % FILENAME)
        exit(1)

    time = data[:, 0]
    # mode = data[:, 1]
    thrust = data[:, 2]
    roll = data[:, 3]
    pitch = data[:, 4]
    yaw = data[:, 5]
    dx = data[:, 6]
    dy = data[:, 7]
    z = data[:, 8]
    dz = data[:, 9]
    # phi = data[:, 10]
    # dphi = data[:, 11]
    # theta = data[:, 12]
    # dtheta = data[:, 13]
    # psi = data[:, 14]
    # dpsi = data[:, 15]

    plt.subplot(4, 1, 1)
    plt.plot(time, thrust, 'b')
    plt.ylim((0, 1))
    plotmid(time, 0.5)
    plt.ylabel('Thrust')

    plt.subplot(4, 1, 2)
    plt.plot(time, z)
    plt.ylim((0, ZMAX))
    plt.ylabel('Z (m)')

    plt.subplot(4, 1, 3)
    plt.plot(time, dx, 'r')
    plt.plot(time, dy, 'g')
    plt.plot(time, dz, 'b')
    plt.ylim((-VELMAX, +VELMAX))
    plt.legend(('dx/dt', 'dy/dt', 'dz/dt'))
    plt.ylabel('Vel (m/s)')

    plt.subplot(4, 1, 4)
    plt.plot(time, roll, 'r')
    plt.plot(time, pitch, 'g')
    plt.plot(time, yaw, 'b')
    plotmid(time, 0)
    plt.legend(('roll', 'pitch', 'yaw'))

    plt.xlabel('time (s)')

    plt.show()


main()
