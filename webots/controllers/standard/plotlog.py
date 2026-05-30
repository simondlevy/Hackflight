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

DETAIL_MIN = 35685
DETAIL_MAX = 35695


def main():

    data = np.loadtxt('log.csv', delimiter=',')

    time = data[:, 0]

    plt.subplot(5, 1, 1)
    plt.plot(time, data[:, 1], 'r')
    plt.plot(time, data[:, 4], 'g')
    plt.ylabel('Altitude (m)')
    plt.legend(('Target', 'Actual'))

    plt.subplot(5, 1, 2)
    plt.plot(time, data[:, 2], 'b')
    plt.ylabel('Altitude PID (m/s)')

    plt.subplot(5, 1, 3)
    plt.plot(time, data[:, 3], 'm')
    plt.ylabel('Climbrate PID')

    plt.subplot(5, 1, 4)
    detail = data[:, 3]
    detail[np.logical_or(detail < DETAIL_MIN, detail > DETAIL_MAX)] = 0
    detail_pos = detail[detail > 0]
    print((np.min(detail_pos), np.max(detail_pos)))
    plt.plot(time, detail, 'm')
    plt.ylim((np.min(detail_pos), np.max(detail_pos)))
    plt.ylim((np.min(detail_pos), np.max(detail_pos)))
    plt.ylabel('Climbrate PID (detail)')

    plt.subplot(5, 1, 5)
    plt.plot(time, data[:, 5], 'c')
    plt.plot(time, np.zeros(len(time)), 'k--')
    plt.ylabel('dZ/dt (m/s)')

    plt.show()



main()
