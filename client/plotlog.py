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
DZMAX = 2.0


def plotmid(time, value):
    plt.plot(time, value * np.ones(len(time)), 'k-')


def main():

    data = np.loadtxt('log.csv', delimiter=',', skiprows=1)

    time = data[:,0]
    thrust = data[:,1]
    roll = data[:,2]
    pitch = data[:,3]
    yaw = data[:,4]
    dx = data[:,5]
    dy = data[:,6]
    z = data[:,7]
    dz = data[:,8]
    phi = data[:,9]
    phi = data[:,10]
    theta = data[:,12]
    theta = data[:,12]
    psi = data[:,13]
    psi = data[:,14]

    plt.subplot(3, 1, 1)
    plt.plot(time, z)
    plt.ylim((0, ZMAX))
    plt.ylabel('Z (m)')

    plt.subplot(3, 1, 2)
    plt.plot(time, dz, 'r')
    plotmid(time, 0)
    plt.ylim((-DZMAX, +DZMAX))
    plt.ylabel('dZ/dt (m/s)')

    plt.subplot(3, 1, 3)
    plt.plot(time, thrust, 'b')
    plt.ylim((0, 1))
    plotmid(time, 0.5)
    plt.ylabel('Thrust')

    plt.xlabel('time (s)')

    plt.show()


main()
