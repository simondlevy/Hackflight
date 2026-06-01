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

ZMAX = 1.5
VELMAX = 1.0
THRUST_TAKEOFF = 0.01
MARGIN = 20


def plotmid(time, value):
    plt.plot(time, value * np.ones(len(time)), 'k-')


def main():

    data = np.loadtxt('log.csv', delimiter=',', skiprows=1)

    thrust = data[:,1]

    start = np.argmax(thrust > THRUST_TAKEOFF) - MARGIN

    time = data[start:,0]
    thrust = data[start:,1]
    roll = data[start:,2]
    pitch = data[start:,3]
    yaw = data[start:,4]
    dx = data[start:,5]
    dy = data[start:,6]
    z = data[start:,7]
    dz = data[start:,8]
    phi = data[start:,9]
    phi = data[start:,10]
    theta = data[start:,12]
    theta = data[start:,12]
    psi = data[start:,13]
    psi = data[start:,14]

    plt.subplot(3, 1, 1)
    plt.plot(time, thrust, 'b')
    plt.ylim((0, 1))
    plotmid(time, 0.5)
    plt.ylabel('Thrust')

    plt.subplot(3, 1, 2)
    plt.plot(time, z)
    plt.ylim((0, ZMAX))
    plt.ylabel('Z (m)')

    plt.subplot(3, 1, 3)
    plt.plot(time, dx, 'r')
    plt.plot(time, dy, 'g')
    plt.plot(time, dz, 'b')
    plotmid(time, 0)
    plt.ylim((-VELMAX, +VELMAX))
    plt.legend(('dx/dt', 'dy/dt', 'dz/dt'))
    plt.ylabel('Vel (m/s)')


    plt.xlabel('time (s)')

    plt.show()


main()
