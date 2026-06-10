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
    m1 = data[:,16]
    m2 = data[:,17]
    m3 = data[:,18]
    m4 = data[:,19]

    beg = (time >= args.begin).argmax()

    end = -1 if args.end is None else (time <= args.end).argmin() 

    plt.subplot(5, 1, 1)
    plt.plot(time[beg:end], thrust[beg:end], 'b')
    plt.ylim((0, 1))
    plotmid(time[beg:end], 0.5)
    plt.ylabel('Thrust')

    plt.subplot(5, 1, 2)
    plt.plot(time[beg:end], z[beg:end])
    plt.ylim((0, args.zmax))
    plt.ylabel('Z (m)')

    plt.subplot(5, 1, 3)
    plt.plot(time[beg:end], dy[beg:end], 'r')
    plt.plot(time[beg:end], dx[beg:end], 'g')
    plt.plot(time[beg:end], dz[beg:end], 'b')
    plt.ylim((-args.vmax, +args.vmax))
    plt.legend(('dy/dt', 'dx/dt', 'dz/dt'))
    plt.ylabel('Vel (m/s)')

    plt.subplot(5, 1, 4)
    plt.plot(time[beg:end], roll[beg:end], 'r')
    plt.plot(time[beg:end], pitch[beg:end], 'g')
    plt.plot(time[beg:end], yaw[beg:end], 'b')
    plotmid(time[beg:end], 0)
    plt.legend(('roll', 'pitch', 'yaw'))
    plt.ylim((-args.smax, +args.smax))
    plt.ylabel('Setpoint')

    plt.subplot(5, 1, 5)
    plt.plot(time[beg:end], m1[beg:end], 'r')
    plt.plot(time[beg:end], m2[beg:end], 'g')
    plt.plot(time[beg:end], m3[beg:end], 'b')
    plt.plot(time[beg:end], m4[beg:end], 'k')
    plotmid(time[beg:end], 0)
    plt.legend(('m1', 'm2', 'm3', 'm4'))
    plt.ylabel('Motors')

    plt.xlabel('time (s)')

    plt.show()


main()
