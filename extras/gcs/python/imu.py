#!/usr/bin/env python
'''
Class or displaying vehicle orientation

Copyright (C) Alec Singer and Simon D. Levy 2021

MIT License
'''

import tkinter as tk
from math import sin, cos
from dialog import Dialog
from vehicle import get_vehicle

LEVEL_MAX_ANGLE_DIFF = .25

VEHICLE_SCALE = 0.10

UPDATE_MSEC = 10

YAW_ACTIVE = 1
PITCH_ACTIVE = 2
ROLL_ACTIVE = 3

# XXX replace with numpy ----------------------------------------


def _eye3():
    return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def _dotv(v, a):
    d = [0, 0, 0]
    d[0] = v[0]*a[0][0] + v[1]*a[1][0] + v[2]*a[2][0]
    d[1] = v[0]*a[0][1] + v[1]*a[1][1] + v[2]*a[2][1]
    d[2] = v[0]*a[0][2] + v[1]*a[1][2] + v[2]*a[2][2]
    return d


def _dotm(a, b):
    _d = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
    for i in range(3):
        for j in range(3):
            for k in range(3):
                _d[i][j] += a[i][k]*b[k][j]
    return _d


def _transpose(a):
    _t = _eye3()
    for j in range(3):
        for k in range(3):
            _t[j][k] = a[k][j]
    return _t

# IMU class ---------------------------------------------------


class IMU(Dialog):

    def __init__(self, driver, simulation=False):

        Dialog.__init__(self, driver)

        # Vehicle dimensions
        W = VEHICLE_SCALE
        D = VEHICLE_SCALE / 2
        L = VEHICLE_SCALE * 2

        # Let these be in World-coordinates (worldview-matrix already applied)
        # In right-handed, counter-clockwise order
        (self.vehicle_points,
         self.vehicle_faces,
         self.vehicle_face_colors) = get_vehicle(W, D, L)

        # Assume no angles to start
        self.roll_pitch_yaw = None

        # Rotation matrices
        self.pitchrot = _eye3()
        self.yawrot = _eye3()
        self.rollrot = _eye3()

        self.simulation = simulation
        self.running = False

    def start(self):

        self.schedule_display_task(UPDATE_MSEC)

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

            self.roll_pitch_yaw = self.driver.getRollPitchYaw()

            self._update()

            self.schedule_display_task(UPDATE_MSEC)

    def _to_screen_coords(self, pv):

        d = str(self.driver.root.geometry()).split('+')[0].split('x')
        dims = [int(s)for s in d]
        width, height = dims[0], dims[1]

        x = width/2*pv[0] + width/2
        y = -height/2*pv[1] + height/2
        z = pv[2]

        return [x, y, z]

    def _save(self):

        self.driver.save(self.pitchroll_kp_scale.get(),
                         self.yaw_kp_scale.get())

    def _create_window(self, x, widget):

        return self.driver.canvas.create_window(x, 10, anchor=tk.NW,
                                                window=widget)

    def _create_button(self, text, x, command):

        button = tk.Button(self.driver.canvas, text=text, height=2,
                           command=command)
        button_window = self._create_window(x, button)

        return button, button_window

    def _create_scale(self, text, x, callback):

        scale = tk.Scale(self.driver.canvas, from_=0, to_=100, label=text,
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

        # Roll-angle comes in as positive for right-side down, so negate for
        # display
        rollAngle = -self.roll_pitch_yaw[0]

        self.rollrot[0][0] = +cos(rollAngle)
        self.rollrot[0][1] = -sin(rollAngle)
        self.rollrot[1][0] = +sin(rollAngle)
        self.rollrot[1][1] = +cos(rollAngle)

        pitchAngle = self.roll_pitch_yaw[1]
        self.pitchrot[1][1] = +cos(pitchAngle)
        self.pitchrot[1][2] = -sin(pitchAngle)
        self.pitchrot[2][1] = +sin(pitchAngle)
        self.pitchrot[2][2] = +cos(pitchAngle)

        yawAngle = -self.roll_pitch_yaw[2]
        self.yawrot[0][0] = +cos(yawAngle)
        self.yawrot[0][2] = +sin(yawAngle)
        self.yawrot[2][0] = -sin(yawAngle)
        self.yawrot[2][2] = +cos(yawAngle)

        # Multiply matrices based on active axis
        if self.driver.active_axis == YAW_ACTIVE:
            rot = _dotm(_dotm(self.rollrot, self.pitchrot), self.yawrot)
        elif self.driver.active_axis == PITCH_ACTIVE:
            rot = _dotm(_dotm(self.yawrot, self.rollrot), self.pitchrot)
        else:
            rot = _dotm(_dotm(self.yawrot, self.pitchrot), self.rollrot)

        # Add a label for arming if needed
        self.driver.checkArmed()

        # Draw polygons
        for i in range(len(self.vehicle_faces)):

            poly = []  # transformed polygon

            for j in range(len(self.vehicle_faces[0])):

                v = self.vehicle_points[self.vehicle_faces[i][j]]

                # Transform the point from 3D to 2D
                ps = _dotv(v, _transpose(rot))
                p = self._to_screen_coords(ps)

                # Put the screenpoint in the list of transformed vertices
                poly.append((p[0], p[1]))

            if self._is_polygon_front_face(poly):  # Backface culling
                f = self.vehicle_face_colors[i]
                self.faces.append(self.driver.canvas.create_polygon(*poly,
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

# Testing =====================================================================


class SliderDriver(object):

    def __init__(self, root, canvas):

        self.root = root
        self.canvas = canvas

        frame = tk.Frame(root)
        pane = tk.PanedWindow(frame)
        pane.pack(fill=tk.BOTH, expand=1)

        self.yaw_scale = self.add_scale(pane, 'Yaw', self.yaw_callback)
        self.pitch_scale = self.add_scale(pane, 'Pitch', self.pitch_callback)
        self.roll_scale = self.add_scale(pane, 'Roll', self.roll_callback)

        tk.Button(pane, text='Reset', command=self.reset).pack()

        pane.pack()
        frame.pack()

        self.reset()

        self.active_axis = 0

    def reset(self):

        self.yaw = self.zero(self.yaw_scale)
        self.pitch = self.zero(self.pitch_scale)
        self.roll = self.zero(self.roll_scale)

    def zero(self, scale):

        scale.set(0)
        return 0

    def add_scale(self, pane, label, callback):

        scale = tk.Scale(pane, label=label, from_=-180, to=180,
                         command=callback, orient=tk.HORIZONTAL, length=200)
        scale.pack()
        return scale

    def yaw_callback(self, valstr):

        self.active_axis = YAW_ACTIVE
        self.yaw = int(valstr)

    def pitch_callback(self, valstr):

        self.active_axis = PITCH_ACTIVE
        self.pitch = int(valstr)

    def roll_callback(self, valstr):

        self.active_axis = ROLL_ACTIVE
        self.roll = int(valstr)

    def getRollPitchYaw(self):

        return self.yaw, self.pitch, self.roll

    def checkArmed(self):

        None


if __name__ == "__main__":

    width = 800
    height = 800

    root = tk.Tk()

    root.geometry('%dx%d+%d+%d' % (width, height+200, 200, 200))
    root.title('IMU')

    canvas = tk.Canvas(root, width=width, height=height, background='black')

    driver = SliderDriver(root, canvas)

    canvas.pack()

    sim = IMU(driver, simulation=True)

    sim.start()

    tk.mainloop()
