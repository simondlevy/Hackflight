'''
This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
'''

'''
Class or displaying vehicle orientation

Copyright (C) Alec Singer and Simon D. Levy 2021

MIT License
'''

import tkinter as tk
import numpy as np
from math import sin, cos
from dialog import Dialog
from vehicle import get_vehicle


class ImuDialog(Dialog):

    def __init__(self, viz, simulation=False, vehicleScale=0.1, updateMsec=10):

        Dialog.__init__(self, viz)

        # Vehicle dimensions
        W = vehicleScale
        D = vehicleScale / 2
        L = vehicleScale * 2

        # Update period
        self.update_msec = updateMsec

        # Let these be in World-coordinates (worldview-matrix already applied)
        # In right-handed, counter-clockwise order
        (self.vehicle_points,
         self.vehicle_faces,
         self.vehicle_face_colors) = get_vehicle(W, D, L)

        # Assume no angles to start
        self.roll_pitch_yaw = None

        # Rotation matrices
        self.pitchrot = np.eye(3)
        self.yawrot = np.eye(3)
        self.rollrot = np.eye(3)

        self.simulation = simulation
        self.running = False

    def start(self):

        self.schedule_display_task(self.update_msec)

        self.running = True

        self.faces = []

        self.roll_pitch_yaw_prev = None
        self.roll_pitch_yaw_change = None

    def stop(self):

        self._clear()
        self.running = False

    def _clear(self):

        for face in self.faces:
            self.delete(face)
        self.faces = []

    def _task(self):

        if self.running:

            self.roll_pitch_yaw = self.viz.getRollPitchYaw()

            self._update()

            self.schedule_display_task(self.update_msec)

    def _to_screen_coords(self, pv):

        d = str(self.viz.root.geometry()).split('+')[0].split('x')
        dims = [int(s)for s in d]
        width, height = dims[0], dims[1]

        x = width/2*pv[0] + width/2
        y = -height/2*pv[1] + height/2
        z = pv[2]

        return [x, y, z]

    def _save(self):

        self.viz.save(self.pitchroll_kp_scale.get(), self.yaw_kp_scale.get())

    def _create_window(self, x, widget):

        return self.viz.canvas.create_window(x, 10, anchor=tk.NW,
                                             window=widget)

    def _create_button(self, text, x, command):

        button = tk.Button(self.viz.canvas, text=text, height=2,
                           command=command)
        button_window = self._create_window(x, button)

        return button, button_window

    def _create_scale(self, text, x, callback):

        scale = tk.Scale(self.viz.canvas, from_=0, to_=100, label=text,
                         command=callback, orient=tk.HORIZONTAL, length=200,
                         bg='black', fg='white')
        scale_window = self._create_window(x, scale)

        return scale, scale_window

    def _pitchroll_kp_scale_callback(self, valstr):

        self.pitchroll_kp_percent = int(valstr)

    def _yaw_kp_scale_callback(self, valstr):

        self.yaw_kp_percent = int(valstr)

    def _update(self):

        # Erase previous image
        self._clear()

        # Convert angles to X,Y,Z rotation matrices

        # Negate incoming angles for display
        rollAngle  = -self.roll_pitch_yaw[0]
        pitchAngle = -self.roll_pitch_yaw[1]
        yawAngle   =  self.roll_pitch_yaw[2]

        self.rollrot[0][0] = +cos(rollAngle)
        self.rollrot[0][1] = -sin(rollAngle)
        self.rollrot[1][0] = +sin(rollAngle)
        self.rollrot[1][1] = +cos(rollAngle)

        self.pitchrot[1][1] = +cos(pitchAngle)
        self.pitchrot[1][2] = -sin(pitchAngle)
        self.pitchrot[2][1] = +sin(pitchAngle)
        self.pitchrot[2][2] = +cos(pitchAngle)

        self.yawrot[0][0] = +cos(yawAngle)
        self.yawrot[0][2] = +sin(yawAngle)
        self.yawrot[2][0] = -sin(yawAngle)
        self.yawrot[2][2] = +cos(yawAngle)

        rot = np.dot(np.dot(self.yawrot, self.pitchrot), self.rollrot)

        # Draw polygons
        for i in range(len(self.vehicle_faces)):

            poly = []  # transformed polygon

            for j in range(len(self.vehicle_faces[0])):

                v = self.vehicle_points[self.vehicle_faces[i][j]]

                # Transform the point from 3D to 2D
                ps = np.dot(v, rot.transpose())
                p = self._to_screen_coords(ps)

                # Put the screenpoint in the list of transformed vertices
                poly.append((p[0], p[1]))

            if self._is_polygon_front_face(poly):  # Backface culling
                f = self.vehicle_face_colors[i]
                self.faces.append(self.viz.canvas.create_polygon(*poly,
                                                                 fill=f))

        # Update angle changes
        if self.roll_pitch_yaw_prev is not None:
            self.roll_pitch_yaw_change = [
                    abs(pair[0]-pair[1])
                    for pair in zip(self.roll_pitch_yaw,
                                    self.roll_pitch_yaw_prev)]
        self.roll_pitch_yaw_prev = self.roll_pitch_yaw

    def _is_polygon_front_face(self, pts):

        summa = 0.0
        num = len(pts)
        for i in range(num-1):
            summa += (pts[i+1][0]-pts[i][0])*(pts[i+1][1]+pts[i][1])

        summa += (pts[0][0]-pts[num-1][0])*(pts[0][1]+pts[num-1][1])

        return summa > 0.0
