#!/usr/bin/env python

# Setup class for displaying vehicle orientation ====================================================

LEVEL_MAX_ANGLE_DIFF = .25

VEHICLE_SCALE = 0.10

UPDATE_MSEC = 10

YAW_ACTIVE = 1
PITCH_ACTIVE = 2
ROLL_ACTIVE = 3


from tk import *

from math import sin, cos, radians, degrees
import numpy as np

from dialog import Dialog

#from blocky_vehicle import get_vehicle
from realistic_vehicle import get_vehicle

class Setup(Dialog):

    def __init__(self, driver, simulation=False):

        Dialog.__init__(self, driver)

        # Vehicle dimensions
        W = VEHICLE_SCALE
        D = VEHICLE_SCALE / 2
        L = VEHICLE_SCALE * 2

        #Let these be in World-coordinates (worldview-matrix already applied)
        ####In right-handed, counter-clockwise order
        self.vehicle_points, self.vehicle_faces, self.vehicle_face_colors = get_vehicle(W, D, L)

        # Assume no angles to start
        self.yaw_pitch_roll = None

        # Rotation matrices
        self.pitchrot = np.eye(3)
        self.yawrot = np.eye(3)
        self.rollrot = np.eye(3)

        self.simulation = simulation
        self.running = False

    def start(self):

        self.schedule_display_task(UPDATE_MSEC)

        self.running = True

        self.faces = []

        self.yaw_pitch_roll_prev = None
        self.yaw_pitch_roll_change = None


    def stop(self):

        self._clear()
        self.running = False

    def _clear(self):

        for face in self.faces:
            self.delete(face)
        self.faces = []

    def _task(self):

        if self.running:

            self.yaw_pitch_roll = self.driver.getYawPitchRoll()

            self._update()

            self.schedule_display_task(UPDATE_MSEC)

    def _to_screen_coords(self, pv):

        dims = [int(s) for s in str(self.driver.root.geometry()).split('+')[0].split('x')]
        width, height = dims[0], dims[1]

        SC = np.eye(4)

        SC[0,0] = width/2
        SC[1,1] = -height/2
        SC[0,3] = width/2
        SC[1,3] = height/2

        x = SC[0,0]*pv[0]+SC[0,1]*pv[1]+SC[0,2]*pv[2]+SC[0,3]
        y = SC[1,0]*pv[0]+SC[1,1]*pv[1]+SC[1,2]*pv[2]+SC[1,3]
        z = SC[2,0]*pv[0]+SC[2,1]*pv[1]+SC[2,2]*pv[2]+SC[2,3]

        return np.array([x, y, z])

    def _save(self):

        self.driver.save(self.pitchroll_kp_scale.get(), self.yaw_kp_scale.get())

    def _create_window(self, x, widget):

        return self.driver.canvas.create_window(x, 10, anchor=NW, window=widget)

    def _create_button(self, text, x, command):

        button = Button(self.driver.canvas, text=text, height=2, command=command);
        button_window = self._create_window(x, button)

        return button, button_window

    def _create_scale(self, text, x, callback):
        
        scale = Scale(self.driver.canvas, from_=0, to_=100, label=text, command=callback,
                orient=HORIZONTAL, length=200, bg='black', fg='white')
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

        yawAngle   = radians(self.yaw_pitch_roll[0])
        self.yawrot[0,0] = +cos(yawAngle)
        self.yawrot[0,2] = +sin(yawAngle)
        self.yawrot[2,0] = -sin(yawAngle)
        self.yawrot[2,2] = +cos(yawAngle)

        pitchAngle = radians(self.yaw_pitch_roll[1]) 
        self.pitchrot[1,1] = +cos(pitchAngle) 
        self.pitchrot[1,2] = -sin(pitchAngle)
        self.pitchrot[2,1] = +sin(pitchAngle)
        self.pitchrot[2,2] = +cos(pitchAngle)

        rollAngle  = -radians(self.yaw_pitch_roll[2]) # negate so positive is roll rightward
        self.rollrot[0,0] = +cos(rollAngle)
        self.rollrot[0,1] = -sin(rollAngle)
        self.rollrot[1,0] = +sin(rollAngle)
        self.rollrot[1,1] = +cos(rollAngle)

        # Multiply matrices based on active axis
        if self.driver.active_axis == YAW_ACTIVE:
            rot = np.dot(np.dot(self.rollrot, self.pitchrot), self.yawrot)
        elif self.driver.active_axis == PITCH_ACTIVE:
            rot = np.dot(np.dot(self.yawrot, self.rollrot), self.pitchrot)
        else:
            rot = np.dot(np.dot(self.yawrot, self.pitchrot), self.rollrot)

        # Add a label for arming if needed
        self.driver.checkArmed()

        # Draw polygons
        for i in range(len(self.vehicle_faces)):

            poly = [] #transformed polygon

            for j in range(len(self.vehicle_faces[0])):

                v = self.vehicle_points[self.vehicle_faces[i][j]]

                # Transform the point from 3D to 2D
                ps = np.dot(v, rot.T)
                p = self._to_screen_coords(ps)
               
                # Put the screenpoint in the list of transformed vertices
                poly.append((p[0], p[1]))

            if self._is_polygon_front_face(poly): #Backface culling

                self.faces.append(self.driver.canvas.create_polygon(*poly, fill=self.vehicle_face_colors[i]))

        # Update angle changes
        if not self.yaw_pitch_roll_prev is None:
            self.yaw_pitch_roll_change = [degrees(abs(pair[0]-pair[1])) 
                    for pair in zip(self.yaw_pitch_roll,self.yaw_pitch_roll_prev)]
        self.yaw_pitch_roll_prev = self.yaw_pitch_roll

    def _is_polygon_front_face(self, pts):

        summa = 0.0
        num = len(pts)
        for i in range(num-1):
            summa += (pts[i+1][0]-pts[i][0])*(pts[i+1][1]+pts[i][1])

        summa += (pts[0][0]-pts[num-1][0])*(pts[0][1]+pts[num-1][1])

        return summa > 0.0

# Testing ==============================================================================================================

class SliderDriver(object):

    def __init__(self, root, canvas):

        self.root = root
        self.canvas = canvas

        frame = Frame(root)
        pane = PanedWindow(frame)
        pane.pack(fill=BOTH, expand=1)

        self.yaw_scale   = self.add_scale(pane, 'Yaw', self.yaw_callback) 
        self.pitch_scale = self.add_scale(pane, 'Pitch', self.pitch_callback) 
        self.roll_scale  = self.add_scale(pane, 'Roll', self.roll_callback) 

        Button(pane, text='Reset', command=self.reset).pack()

        pane.pack()
        frame.pack()

        self.reset()

        self.active_axis = 0

   
    def reset(self):

        self.yaw   = self.zero(self.yaw_scale)
        self.pitch = self.zero(self.pitch_scale)
        self.roll  = self.zero(self.roll_scale)

    def zero(self, scale):

        scale.set(0)
        return 0

    def add_scale(self, pane, label, callback):

        scale =Scale(pane, label=label, from_=-180, to=180, command=callback, orient=HORIZONTAL, length=200)
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

    def getYawPitchRoll(self):

        return self.yaw, self.pitch, self.roll

    def checkArmed(self):

        None

if __name__ == "__main__":

    width = 800
    height = 800

    root = Tk()

    root.geometry('%dx%d+%d+%d' % (width, height+200, 200, 200))
    root.title('Setup')

    canvas = Canvas(root, width=width, height=height, background='black')

    driver = SliderDriver(root, canvas)

    canvas.pack()

    sim = Setup(driver, simulation=True)

    sim.start()

    mainloop()
