#!/usr/bin/env python3
'''
randomwalk.py: Generate a 3D random walk to test visualization

Copyright (C) 2019 Simon D. Levy

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

Hackflight is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http:#www.gnu.org/licenses/>.
'''

TRANSLATE_SCALE = 0.1
ROTATE_SCALE    = 10

from time import time
import numpy as np
import argparse

class _MyArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(1)

if __name__ == '__main__':

    # Parse optional command-line arguments
    parser = _MyArgumentParser(description='Visualize a random walk.')
    parser.add_argument('-s', '--seed', help='set seed for pseudo-random number generator')
    cmdargs = parser.parse_args()

    # Set seed for pseudo-random number generator if indicated
    if not cmdargs.seed is None:
        np.random.seed(int(cmdargs.seed))

    # Start in the center of the map with a random heading
    pose = np.array([0,0,0,360*np.random.random()])

    # Loop till user closes the display
    while True:

               
        try:

            # Rotate randomly and move forward
            theta = np.radians(pose[3])
            pose[0] += TRANSLATE_SCALE * np.cos(theta)
            pose[1] += TRANSLATE_SCALE * np.sin(theta)
            pose[2] += TRANSLATE_SCALE * np.random.randn()
            pose[3] += ROTATE_SCALE * np.random.randn()
                 
            print('%+3.3f %+3.3f %+3.3f %+3.3f' % tuple(pose))

        except KeyboardInterrupt:

            break


