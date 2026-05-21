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

def main():

    data = np.loadtxt('log.csv', delimiter=',', skiprows=1)

    t = data[:,0] - data[0, 0]

    plt.subplot(2, 1, 1)
    plt.plot(t, data[:,3])
    plt.ylim((0, ZMAX))
    plt.ylabel('Z (m)')

    plt.subplot(2, 1, 2)
    plt.plot(t, data[:,4], 'r')
    plt.plot(t, np.zeros(len(t)), 'k')
    plt.ylim((-DZMAX, +DZMAX))
    plt.ylabel('dZ/dt (m/s)')

    plt.xlabel('time (s)')

    plt.show()


main()
