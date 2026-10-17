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


def plotmid(time, value):
    plt.plot(time, value * np.ones(len(time)), 'k-')


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
    # volts = data[:, 1]
    # thrust = data[:, 2]
    roll = data[:, 3]
    pitch = data[:, 4]
    yaw = data[:, 5]
    dx = data[:, 6]
    dy = data[:, 7]
    z = data[:, 8]
    dz = data[:, 9]
    phi = data[:, 10]
    # dphi = data[:, 11]
    theta = data[:, 12]
    # dtheta = data[:, 13]
    # psi = data[:, 14]
    dpsi = data[:, 15]
    m1 = data[:, 16]
    m2 = data[:, 17]
    m3 = data[:, 18]
    m4 = data[:, 19]

    beg = (time >= args.begin).argmax()

    end = -1 if args.end is None else (time <= args.end).argmin()

    t = time[beg:end]

    plt.subplot(6, 1, 1)
    plt.plot(t, z[beg:end])
    plt.plot(t, dz[beg:end])
    plt.legend(('z', 'dz/dt'))
    plt.ylabel('Climb')

    plt.subplot(6, 1, 2)
    plt.plot(t, 'r')
    plt.plot(t, 'g')
    plt.plot(t, 'b')
    plotmid(t, 0)
    plt.legend(('roll', 'pitch', 'yaw'))
    plt.ylabel('Setpoint')

    plt.subplot(6, 1, 3)
    plt.plot(t, 'r')
    plt.plot(t, 'g')
    plotmid(t, 0)
    plt.legend(('phi', 'theta'))
    plt.ylabel('Angles')

    plt.subplot(6, 1, 4)
    plt.plot(t, dpsi[beg:end])
    plotmid(t, 0)
    plt.ylabel('dpsi/dt')

    plt.subplot(6, 1, 5)
    plt.plot(t, 'r')
    plt.plot(t, 'g')
    plt.ylim((-args.vmax, +args.vmax))
    plt.legend(('dy/dt', 'dx/dt'))
    plt.ylabel('Vel (m/s)')

    plt.subplot(6, 1, 6)
    plt.plot(t, m1[beg:end], '+')
    plt.plot(t, m2[beg:end], '+')
    plt.plot(t, m3[beg:end])
    plt.plot(t, m4[beg:end])
    plt.legend(('rr', 'rf', 'lr', 'lf'))
    plt.ylabel('Motors')

    plt.xlabel('time (s)')

    plt.show()


main()
