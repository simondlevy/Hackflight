#!/usr/bin/env python3
'''
Generate a 3D random walk to test visualization

Copyright (C) 2021 Simon D. Levy

MIT License
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


