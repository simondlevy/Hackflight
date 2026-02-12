'''
roboviz.py - Python classes for displaying maps and robots

Requires: numpy, matplotlib

Copyright (C) 2018 Simon D. Levy

This file is part of PyRoboViz.

PyRoboViz is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

PyRoboViz is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
'''

# Essential imports
import matplotlib.pyplot as plt
import matplotlib.cm as colormap
import matplotlib.lines as mlines
import numpy as np

# Use native macOS backend; fall back to TkAgg on other platforms
import matplotlib
import platform
if platform.system() == 'Darwin':
    matplotlib.use('macosx')
else:
    matplotlib.use('TkAgg')


class Visualizer(object):

    # Robot display params
    ROBOT_HEIGHT_M = 0.5
    ROBOT_WIDTH_M = 0.3

    def __init__(self, map_size_meters, map_size_pixels=800):

        # Store constants for display
        self.map_size_meters = map_size_meters
        self.map_size_pixels = map_size_pixels

        # Create a byte array to display the map with a color overlay
        self.bgrbytes = bytearray(map_size_pixels * map_size_pixels * 3)

        # Make a nice big (10"x10") figure
        fig = plt.figure(figsize=(10, 10))

        # Store Python ID of figure to detect window close
        self.figid = id(fig)

        # Use an "artist" to speed up map drawing
        self.img_artist = None

        # No vehicle to show yet
        self.vehicle = None

        # Create axes
        self.ax = fig.gca()
        self.ax.grid(False)

        # Store previous position for trajectory
        self.prevpos = None

        self.rotate_angle = 0

    def display(self, x_m, y_m, theta_deg,
                start_angle=0,
                title='',
                flip_axes=False,
                map_bytes=None,
                show_trajectory=False,
                obstacles=[]):

        # print(x_m, y_m)

        self._set_pose(x_m, y_m, theta_deg, start_angle,
                      show_trajectory, flip_axes)

        shift = -self.map_size_pixels / 2 if map_bytes is None else 0

        # We base the axis on pixels, to support displaying the map
        self.ax.set_xlim([shift, self.map_size_pixels+shift])
        self.ax.set_ylim([shift, self.map_size_pixels+shift])

        if map_bytes is not None:
            self._showMap(map_bytes)

        self._show_obstacles(obstacles, flip_axes)

        plt.title(title)

        self.ax.set_xlabel('Y (mm)' if flip_axes else 'X (mm)')
        self.ax.set_ylabel('X (mm)' if flip_axes else 'Y (mm)')

        return self._refresh()

    def _showMap(self, map_bytes):

        mapimg = np.reshape(np.frombuffer(map_bytes, dtype=np.uint8),
                            (self.map_size_pixels, self.map_size_pixels))

        if self.img_artist is None:

            self.img_artist = self.ax.imshow(mapimg, cmap=colormap.gray)

        else:

            self.img_artist.set_data(mapimg)

    def _show_obstacles(self, obstacles, flip_axes):

        for obst in obstacles:
            xs = [x * 100 for x in obst['x']] # (0, 100, 100, 0, 0)
            ys = [y * 100 for y in obst['y']] # (0, 0, 100, 100, 0)
            plt.fill(xs, ys, color='black')


    def _set_pose(self, x_m, y_m, theta_deg, start_angle, showtraj, flip_axes):

        # If zero-angle was indicated, grab first angle to compute rotation
        if start_angle is None and self.zero_angle != 0:
            start_angle = theta_deg
            self.rotate_angle = self.zero_angle - self.start_angle

        # Flip axes if indicated
        if flip_axes:
            x_m, y_m = y_m, x_m
            theta_deg = 90 - theta_deg

        # Rotate by computed angle, or zero if no zero-angle indicated
        d = self.rotate_angle
        a = np.radians(d)
        c = np.cos(a)
        s = np.sin(a)
        x_m, y_m = x_m*c-y_m*s, y_m*c+x_m*s

        # Erase previous vehicle image after first iteration
        if self.vehicle is not None:
            self.vehicle.remove()

        # Use a very short arrow shaft to orient the head of the arrow
        theta_rad = np.radians(theta_deg+d)
        c = np.cos(theta_rad)
        s = np.sin(theta_rad)
        L = 0.1
        dx = L * c
        dy = L * s

        s = self.map_size_meters / self.map_size_pixels

        self.vehicle = self.ax.arrow(x_m/s, y_m/s, dx, dy,
                                     head_width=Visualizer.ROBOT_WIDTH_M/s,
                                     head_length=Visualizer.ROBOT_HEIGHT_M/s,
                                     fc='r', ec='r')

        # Show trajectory if indicated
        currpos = x_m/s, y_m/s
        if showtraj and self.prevpos is not None:
            self.ax.add_line(mlines.Line2D((self.prevpos[0], currpos[0]),
                             (self.prevpos[1], currpos[1])))
        self.prevpos = currpos

    def _refresh(self):

        # If we have a new figure, something went wrong (closing figure failed)
        if self.figid != id(plt.gcf()):
            return False

        # Redraw current objects without blocking
        plt.draw()

        # Refresh display, setting flag on window close or keyboard interrupt
        try:
            plt.pause(.01)  # Arbitrary pause to force redraw
            return True
        except Exception:
            return False

        return True
