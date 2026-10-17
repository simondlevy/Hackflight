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
import argparse
from argparse import ArgumentDefaultsHelpFormatter


def plotmid(time, value=0):
    plt.plot(time, value * np.ones(len(time)), 'k--')


def main():

    argparser = argparse.ArgumentParser(
            formatter_class=ArgumentDefaultsHelpFormatter)

    argparser.add_argument('-i', '--infile', default='log.csv',
                           help='CSV input file')

    argparser.add_argument('-b', '--begin', type=float, default=0,
                           help='begin time in seconds')

    argparser.add_argument('-e', '--end', type=float, default=None,
                           help='end time in seconds')

    argparser.add_argument('-z', '--zmax', type=float, default=1.0,
                           help='maximum altitude (meters)')

    argparser.add_argument('-v', '--vmax', type=float, default=1.5,
                           help='maximum velocity (meters per second)')

    argparser.add_argument('-s', '--smax', type=float, default=0.05,
                           help='setpoint maximum')

    args = argparser.parse_args()

    try:
        data = np.loadtxt(args.infile, delimiter=',', skiprows=1)
    except FileNotFoundError:
        print('File %s not found' % args.infile)
        exit(1)

    time = data[:, 0]

    beg = (time >= args.begin).argmax()

    end = -1 if args.end is None else (time <= args.end).argmin()

    t = time[beg:end]

    volts = data[:, 1]

    throttle = data[beg:end, 2]
    roll = data[beg:end, 3]
    pitch = data[beg:end, 4]
    yaw = data[beg:end, 5]

    dx = data[beg:end, 6]
    dy = data[beg:end, 7]
    z = data[beg:end, 8]
    dz = data[beg:end, 9]
    phi = data[beg:end, 10]
    dphi = data[beg:end, 11]
    theta = data[beg:end, 12]
    dtheta = data[beg:end, 13]
    psi = data[beg:end, 14]
    dpsi = data[beg:end, 15]

    m1 = data[beg:end, 16]
    m2 = data[beg:end, 17]
    m3 = data[beg:end, 18]
    m4 = data[beg:end, 19]


    # Stick demands
    plt.subplot(3, 1, 1)
    plt.plot(t, throttle, 'r--')
    plt.plot(t, roll, 'g')
    plt.plot(t, pitch, 'b')
    plt.plot(t, yaw, 'c')
    plotmid(t)
    plt.ylim((-1,+1))
    plt.legend(('throttle', 'roll', 'pitch', 'yaw'))
    plt.ylabel('Sticks')

    # Roll, pitch angles
    plt.subplot(3, 1, 2)
    plt.plot(t, phi, 'r')
    plt.plot(t, theta, 'g')
    plotmid(t)
    plt.ylim((-30, +30))
    plt.legend(('phi', 'theta'))
    plt.ylabel('Angles')

    #  Motors
    plt.subplot(3, 1, 3)
    plt.plot(t, m1, '+')
    plt.plot(t, m2, '+')
    plt.plot(t, m3)
    plt.plot(t, m4)
    plt.ylim((0, 1))
    plt.legend(('right-rear', 'right-front', 'left-rear', 'left-front'))
    plt.ylabel('Motors')

    '''
    # Yaw velocity
    plt.subplot(3, 1, 3)
    plt.plot(t, dpsi)
    plotmid(t)
    plt.ylabel('dpsi/dt')

    # Altitude, climbrate
    plt.subplot(3, 1, 1)
    plt.plot(t, z)
    plt.plot(t, dz)
    plt.legend(('z', 'dz/dt'))
    plt.ylim((-2, +2))
    plt.ylabel('Climb')

    # Lateral velocity
    plt.subplot(6, 1, 5)
    plt.plot(t, 'r')
    plt.plot(t, 'g')
    plt.ylim((-args.vmax, +args.vmax))
    plt.legend(('dy/dt', 'dx/dt'))
    plt.ylabel('Vel (m/s)')

    '''

    plt.xlabel('time (s)')

    plt.show()


main()
