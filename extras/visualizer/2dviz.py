#!/usr/bin/env python3
'''
2D state visualizer 

Dependencies: numpy, matplotlib, pyserial, pybluez, https://github.com/simondlevy/PyRoboViz

Copyright (C) 2021 Simon D. Levy

MIT License
'''

from  roboviz import Visualizer
import stateviz

MAP_SIZE_PIXELS = 800
MAP_SIZE_METERS = 32

class TwoDVisualizer(Visualizer):

    def __init__(self, cmdargs, label, outfile=None):

        zero_angle = float(cmdargs.zero_angle) if not cmdargs.zero_angle is None else 0

        Visualizer.__init__(self, MAP_SIZE_PIXELS, MAP_SIZE_METERS, label, True, zero_angle)

        self.outfile = outfile

    def display(self, x_m, y_m, z_m, theta_deg):

        if not self.outfile is None:

            self.outfile.write('%+3.3f %+3.3f %+3.3f %3.3f\n' % (altitude, x_m, y_m, theta_deg))
            self.outfile.flush()

        # Ignore altitude for 2D display
        return Visualizer.display(self, x_m, y_m, theta_deg)

# We pass the class, rather than an instance, because Stateviz.run() will create the instance for us
stateviz.run(TwoDVisualizer)
