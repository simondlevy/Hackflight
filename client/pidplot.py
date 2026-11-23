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

    args = argparser.parse_args()

    try:
        data = np.loadtxt(args.infile, delimiter=',')
    except FileNotFoundError:
        print('File %s not found' % args.infile)
        exit(1)

    time = data[:, 0]

    plt.subplot(4, 1, 1)
    plt.plot(time, data[:, 1])
    plt.plot(time, data[:, 2])
    plt.plot(time, data[:, 3])
    plt.legend(('target', 'actual', 'error'))

    plt.subplot(4, 1, 2)
    plt.plot(time, data[:, 4])
    plt.ylabel('integral')

    plt.subplot(4, 1, 3)
    plt.plot(time, data[:, 5])
    plt.ylabel('derivative')

    plt.subplot(4, 1, 4)
    plt.plot(time, data[:, 6])
    plt.ylabel('output')

    plt.xlabel('time (s)')

    plt.show()


main()
