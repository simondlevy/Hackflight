#!/usr/bin/env python3

'''
Copyright (C) 2026 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''


from roboviz import Visualizer # MapVisualizer
from simsensors.parsers.webots.world import parse

import numpy as np
from sys import argv
from time import sleep
from pprint import pprint

FRAMES_PER_SECOND = 30

LOG_NAME = 'log.csv'

def main():

    if len(argv) < 2:
        print('Usage: %s WORLD_FILE' % argv[0])
        exit(1)
	    
    world = parse(argv[1])

    pprint(world)

    exit(0)

    data = np.loadtxt(LOG_NAME, delimiter=',')

    viz = Visualizer(10)

    for row in data:

        x, y, psi = row[0], row[1], row[5] * 180 / np.pi

        lidar = row[6:]

        if not viz.display(-y, x, 90-psi):
            break

        sleep(1 / FRAMES_PER_SECOND)
                    
main()
