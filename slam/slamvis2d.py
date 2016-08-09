#!/usr/bin/env python

'''
slamvis2d.py : 2D SLAM visualization in Python

Copyright (C) Simon D. Levy 2016

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

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D      
from math import cos, sin, radians

class TwoDSlamVis(object):

    def __init__(self, map_size_cm=1000, vehicle_size_cm=25):
        '''
        Creates a new 2D SLAM visualization.
        '''

        self.vehicle_size_cm = vehicle_size_cm

        # Make a nice big (10"x10") figure
        fig = plt.figure(figsize=(10,10))

        # Store Python ID of figure to detect window close
        self.figid = id(fig)

        fig.canvas.set_window_title('SLAM 2D')

        self.ax = fig.gca()
        self.ax.set_aspect("auto")
        self.ax.set_autoscale_on(True)

        self.ax.set_xlim([-map_size_cm/2, map_size_cm/2])
        self.ax.set_ylim([-map_size_cm/2, map_size_cm/2])

        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')

        self.ax.grid(False)

        self._add_vehicle(0,0,0)

    def addObstacle(self, x, y, phi, s):
        '''
        Adds an obstacle:
        x,y   coordinate of obstacle end
        phi:  rotation angle of obstacle (degrees)
        s:    obstacle size (cm)
        '''

        phi = radians(phi)

        xs = [x, x+s*cos(phi)]
        ys = [y, y+s*sin(phi)]

        self.ax.add_line(Line2D(xs, ys))

    def setPose(self, x, y, theta):
        '''
        Sets vehicle pose: 
        X:     left/right   (cm)
        Y:     forward/back (cm)
        theta: rotation (degrees)
        '''

        # remove old arrow
        self.vehicle.remove()

        # create a new arrow
        self._add_vehicle(x, y, theta)
    
    def redraw(self):
        '''
        Redraws the display.  You should call this at regular intevals.
        '''

        # If we have a new figure, something went wrong (closing figure failed)
        if self.figid != id(plt.gcf()):
            return False

        # Redraw current objects without blocking
        plt.draw()

        # Refresh display, setting flag on window close or keyboard interrupt
        try:
            plt.pause(.01)
        except:
            return False

        return True 

    def _add_vehicle(self, x, y, theta):

        # Convert vehicle size to length, width, height
        l = self.vehicle_size_cm
        w = l / 2

        # Use a very short arrow shaft to orient the head of the arrow
        r = .01
        theta = radians(theta)
        dx = r * cos(theta)
        dy = r * sin(theta)

        self.vehicle = self.ax.arrow(x, y, dx, dy, head_width=w, head_length=l, fc='r', ec='r')

if __name__ == '__main__':

    from random import uniform
    from time import sleep

    mapsize_cm = 300

    slamvis = TwoDSlamVis(map_size_cm=mapsize_cm)

    x,y,theta = 0,0,0

    while True:

        slamvis.addObstacle(100, 100, 0, 25)

        slamvis.setPose(x,y,theta)

        if not slamvis.redraw():
            break

        sleep(.5)

        theta = (theta + 10) % 360
