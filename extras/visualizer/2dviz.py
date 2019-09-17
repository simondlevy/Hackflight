#!/usr/bin/env python3
'''
2D state visualizer 

Dependencies: numpy, matplotlib, pyserial, pybluez, https://github.com/simondlevy/PyRoboViz

Copyright (C) 2018 Simon D. Levy

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.
This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http:#www.gnu.org/licenses/>.
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
