#!/usr/bin/env python3

import stateviz

class DebugVisualizer(object):

    def __init__(self, cmdargs, label, outfile=None):

        self.outfile = outfile

    def display(self, x_m, y_m, z_m, theta_deg):

        print(x_m, y_m, z_m, theta_deg)

# We pass the class, rather than an instance, because Stateviz.run() will create the instance for us
stateviz.run(DebugVisualizer)
