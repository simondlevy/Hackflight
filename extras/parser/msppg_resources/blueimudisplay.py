#!/usr/bin/env python3

'''
blueimudisplay.py - graphical demo of MSPPG Attitude messages over Bluetooth

Copyright (C) Alec Singer and Simon D. Levy 2016

This code is free software: you can redistribute it and/or modify
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

VEHICLE_SCALE   = 0.10
UPDATE_MSEC     = 10
BT_ADDR         = "00:06:66:73:e3:a6"
BT_PORT         = 1

import tkinter
import socket
import threading
import msppg
from math import sin, cos, radians, degrees
import numpy as np

class Display(object):

    def __init__(self, driver, simulation=False):

        self.driver = driver

        self.width = int(self.driver.canvas['width'])
        self.height = int(self.driver.canvas['height'])

        self.driver.root.bind("<Key>", self._check_quit)

        self.driver.root.title('IMU Telemetry')

        # Vehicle dimensions
        W = VEHICLE_SCALE
        D = VEHICLE_SCALE / 2
        L = VEHICLE_SCALE * 2

        #Let these be in World-coordinates (worldview-matrix already applied)
        ####In right-handed, counter-clockwise order
        self.vehicle_points, self.vehicle_faces, self.vehicle_face_colors = self._get_vehicle(W, D, L)

        # Assume no angles to start
        self.yaw_pitch_roll = None

        # Rotation matrices
        self.pitchrot = np.eye(3)
        self.yawrot = np.eye(3)
        self.rollrot = np.eye(3)

        self.simulation = simulation
        self.running = False

    def start(self, delay_msec=UPDATE_MSEC):

        self._schedule_display_task(delay_msec)

        self.running = True

        self.faces = []

        self.yaw_pitch_roll_prev = None
        self.yaw_pitch_roll_change = None


    def stop(self):

        self._clear()
        self.running = False

    def setParams(self, pitchroll_kp_percent, yaw_kp_percent):

        self.pitchroll_kp_percent = pitchroll_kp_percent
        self.yaw_kp_percent = yaw_kp_percent

        self._set_sliders()

    def _schedule_display_task(self, delay_msec):

        self.driver.root.after(delay_msec, self._task)
        
    def _clear(self):

        for face in self.faces:
            self.driver.canvas.delete(face)

        self.faces = []

    def _task(self):

        if self.running:

            self.yaw_pitch_roll = self.driver.getYawPitchRoll()

            self._update()

            self._schedule_display_task(UPDATE_MSEC)

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

    def _create_window(self, x, widget):

        return self.driver.canvas.create_window(x, 10, anchor=tkinter.NW, window=widget)

    def _check_quit(self, event):

        if ord(event.char) == 27: # ESC
            exit(0)        

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

        # Multiply matrices
        rot = np.dot(np.dot(self.yawrot, self.pitchrot), self.rollrot)

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

    def _get_vehicle(self, width, depth, length):

        #creates constants
        length = width

        #arm constants
        armLength = width*2
        armWidth = armLength/10
        
        #arrow constants
        arrowWidth  = 1.0  * armWidth
        arrowLength = 5.0  * armWidth
        arrowHeight = 1.5  * depth

        #prop constants
        propWidth        = 1.00 * armWidth
        propNarrowWidth  = 0.20 * propWidth
        propLength       = 7.50 * propWidth
        propNarrowLength = 0.75 * propLength
        propShortLength  = 0.25 * propLength

        #prop pitch constants
        tipTU   = 0.900 * depth
        tipTL   = 0.625 * depth
        tipBU   = 0.625 * depth
        tipBL   = 0.350  * depth

        endT    = .75 * depth
        endB    = .50 * depth

        constant1 = ((endT-tipTL)/3) * depth
        constant2 = ((endB-tipBL)/3) * depth

        farTU   = tipTU - constant2
        farTL   = tipTL + constant1
        farBU   = tipBU - constant1
        farBL   = tipBL + constant2
        
        closeTU = farTU - constant2
        closeTL = farTL + constant1
        closeBU = farBU - constant1
        closeBL = farBL + constant2

        points = np.array([

                #creates arm 1
                [+width - armWidth, +depth/2 , +length + armWidth], #0   0
                [+width + armWidth, +depth/2 , +length - armWidth], #1   1
                [+width + armWidth, -depth/2 , +length - armWidth], #2   2
                [+width - armWidth, -depth/2 , +length + armWidth], #3   3

                [+width + armLength - armWidth, +depth/2 , +length + armLength + armWidth], #4   4
                [+width + armLength + armWidth, +depth/2 , +length + armLength - armWidth], #5   5
                [+width + armLength + armWidth, -depth/2 , +length + armLength - armWidth], #6   6
                [+width + armLength - armWidth, -depth/2 , +length + armLength + armWidth], #7   7
                
                #creates arm 2
                [-width - armWidth, +depth/2 , +length - armWidth], #0   8
                [-width + armWidth, +depth/2 , +length + armWidth], #1   9
                [-width + armWidth, -depth/2 , +length + armWidth], #2   10
                [-width - armWidth, -depth/2 , +length - armWidth], #3   11

                [-width - armLength - armWidth, +depth/2 , +length + armLength - armWidth], #4   12
                [-width - armLength + armWidth, +depth/2 , +length + armLength + armWidth], #5   13
                [-width - armLength + armWidth, -depth/2 , +length + armLength + armWidth], #6   14
                [-width - armLength - armWidth, -depth/2 , +length + armLength - armWidth], #7   15

                #creates arm 3
                [+width + armLength - armWidth, +depth/2 , -length - armLength - armWidth], #0   16
                [+width + armLength + armWidth, +depth/2 , -length - armLength + armWidth], #1   17
                [+width + armLength + armWidth, -depth/2 , -length - armLength + armWidth], #2   18
                [+width + armLength - armWidth, -depth/2 , -length - armLength - armWidth], #3   19

                [+width - armWidth, +depth/2 , -length - armWidth], #4   20
                [+width + armWidth, +depth/2 , -length + armWidth], #5   21
                [+width + armWidth, -depth/2 , -length + armWidth], #6   22
                [+width - armWidth, -depth/2 , -length - armWidth], #7   23

                #creates arm 4
                [-width - armLength - armWidth, +depth/2 , -length - armLength + armWidth], #0   24
                [-width - armLength + armWidth, +depth/2 , -length - armLength - armWidth], #1   25
                [-width - armLength + armWidth, -depth/2 , -length - armLength - armWidth], #2   26
                [-width - armLength - armWidth, -depth/2 , -length - armLength + armWidth], #3   27
                
                [-width - armWidth, +depth/2 , -length + armWidth], #4   28
                [-width + armWidth, +depth/2 , -length - armWidth], #5   29
                [-width + armWidth, -depth/2 , -length - armWidth], #6   30
                [-width - armWidth, -depth/2 , -length + armWidth], #7   31

                #creates the arrow body
                [-arrowWidth, +arrowHeight, 0], #0   32
                [+arrowWidth, +arrowHeight, 0], #1   33
                [+arrowWidth, +depth, 0],       #2   34
                [-arrowWidth, +depth, 0],       #3   35

                [-arrowWidth, +arrowHeight, +arrowLength], #4  36
                [+arrowWidth, +arrowHeight, +arrowLength], #5  37
                [+arrowWidth, +depth, +arrowLength],       #6  38
                [-arrowWidth, +depth, +arrowLength],       #7  39
                
                #creates the arrow head
                [-(1/6)*arrowWidth, +arrowHeight, -arrowLength], #0  40
                [+(1/6)*arrowWidth, +arrowHeight, -arrowLength], #1  41
                [+(1/6)*arrowWidth, +depth, -arrowLength],       #2  42
                [-(1/6)*arrowWidth, +depth, -arrowLength],       #3  43
                
                [-arrowWidth - 2*arrowWidth, +arrowHeight, 0],   #4  44
                [+arrowWidth + 2*arrowWidth, +arrowHeight, 0],   #5  45
                [+arrowWidth + 2*arrowWidth, +depth, 0],         #6  46
                [-arrowWidth - 2*arrowWidth, +depth, 0],         #7  47

                #creates the center box
                [-width - armWidth, +depth, -length + armWidth], #0  48
                [-width + armWidth, +depth, -length - armWidth], #1  49
                
                [+width - armWidth, +depth, -length - armWidth], #2  50
                [+width + armWidth, +depth, -length + armWidth], #3  51
                
                [+width - armWidth, -depth, -length - armWidth], #4  52
                [+width + armWidth, -depth, -length + armWidth], #5  53
                
                [-width - armWidth, -depth, -length + armWidth], #6  54
                [-width + armWidth, -depth, -length - armWidth], #7  55
                
                [-width - armWidth, +depth, +length - armWidth], #8  56
                [-width + armWidth, +depth, +length + armWidth], #9  57
                
                [+width - armWidth, +depth, +length + armWidth], #10 58
                [+width + armWidth, +depth, +length - armWidth], #11 59
                
                [+width - armWidth, -depth, +length + armWidth], #12 60
                [+width + armWidth, -depth, +length - armWidth], #13 61
                
                [-width - armWidth, -depth, +length - armWidth], #14 62
                [-width + armWidth, -depth, +length + armWidth], #15 63

                #creates prop 1 on arm 1

                #North East far narrow tip
                [+width+armLength + propLength - propNarrowWidth, +tipTL, +length+armLength - propLength - propNarrowWidth],     #0  64
                [+width+armLength + propLength + propNarrowWidth, +tipTU, +length+armLength - propLength + propNarrowWidth],     #1  65
                [+width+armLength + propLength + propNarrowWidth, +tipBU, +length+armLength - propLength + propNarrowWidth],     #2  66
                [+width+armLength + propLength - propNarrowWidth, +tipBL, +length+armLength - propLength - propNarrowWidth],     #3  67
                #North East far wide
                [+width+armLength + propNarrowLength - propWidth, +farTL, +length+armLength - propNarrowLength - propWidth],     #4  68
                [+width+armLength + propNarrowLength + propWidth, +farTU, +length+armLength - propNarrowLength + propWidth],     #5  69
                [+width+armLength + propNarrowLength + propWidth, +farBU, +length+armLength - propNarrowLength + propWidth],     #6  70
                [+width+armLength + propNarrowLength - propWidth, +farBL, +length+armLength - propNarrowLength - propWidth],     #7  71
                #North East close wide
                [+width+armLength + propShortLength - propWidth, +closeTL, +length+armLength - propShortLength - propWidth],     #4  72
                [+width+armLength + propShortLength + propWidth, +closeTU, +length+armLength - propShortLength + propWidth],     #5  73
                [+width+armLength + propShortLength + propWidth, +farBU, +length+armLength - propShortLength + propWidth],       #6  74
                [+width+armLength + propShortLength - propWidth, +farBL, +length+armLength - propShortLength - propWidth],       #7  75

                #Middle narrow tip
                [+width+armLength - propNarrowWidth, +endT, +length+armLength - propNarrowWidth],    #4  76
                [+width+armLength + propNarrowWidth, +endT, +length+armLength + propNarrowWidth],    #5  77
                [+width+armLength + propNarrowWidth, +endB, +length+armLength + propNarrowWidth],    #6  78
                [+width+armLength - propNarrowWidth, +endB, +length+armLength - propNarrowWidth],    #7  79

                #South West close wide
                [+width+armLength - propShortLength - propWidth, +closeTU, +length+armLength + propShortLength - propWidth],     #4  80
                [+width+armLength - propShortLength + propWidth, +closeTL, +length+armLength + propShortLength + propWidth],     #5  81
                [+width+armLength - propShortLength + propWidth, +closeBL, +length+armLength + propShortLength + propWidth],     #6  82
                [+width+armLength - propShortLength - propWidth, +closeBU, +length+armLength + propShortLength - propWidth],     #7  83
                #South West far wide
                [+width+armLength - propNarrowLength - propWidth, +farTU, +length+armLength + propNarrowLength - propWidth],     #4  84
                [+width+armLength - propNarrowLength + propWidth, +farTL, +length+armLength + propNarrowLength + propWidth],     #5  85
                [+width+armLength - propNarrowLength + propWidth, +farBL, +length+armLength + propNarrowLength + propWidth],     #6  86
                [+width+armLength - propNarrowLength - propWidth, +farBU, +length+armLength + propNarrowLength - propWidth],     #7  87
                #South West far narrow tip
                [+width+armLength - propLength - propNarrowWidth, +tipTU, +length+armLength + propLength - propNarrowWidth],     #0  88
                [+width+armLength - propLength + propNarrowWidth, +tipTL, +length+armLength + propLength + propNarrowWidth],     #1  89
                [+width+armLength - propLength + propNarrowWidth, +tipBL, +length+armLength + propLength + propNarrowWidth],     #2  90
                [+width+armLength - propLength - propNarrowWidth, +tipBU, +length+armLength + propLength - propNarrowWidth],     #3  91

                #creates prop 4 on arm 4

                #North East far narrow tip
                [-width-armLength + propLength - propNarrowWidth, +tipTL, -length-armLength - propLength - propNarrowWidth],     #0  92
                [-width-armLength + propLength + propNarrowWidth, +tipTU, -length-armLength - propLength + propNarrowWidth],     #1  93
                [-width-armLength + propLength + propNarrowWidth, +tipBU, -length-armLength - propLength + propNarrowWidth],     #2  94
                [-width-armLength + propLength - propNarrowWidth, +tipBL, -length-armLength - propLength - propNarrowWidth],     #3  95
                #North East far wide
                [-width-armLength + propNarrowLength - propWidth, +farTL, -length-armLength - propNarrowLength - propWidth],     #4  96
                [-width-armLength + propNarrowLength + propWidth, +farTU, -length-armLength - propNarrowLength + propWidth],     #5  97
                [-width-armLength + propNarrowLength + propWidth, +farBU, -length-armLength - propNarrowLength + propWidth],     #6  98
                [-width-armLength + propNarrowLength - propWidth, +farBL, -length-armLength - propNarrowLength - propWidth],     #7  99
                #North East close wide
                [-width-armLength + propShortLength - propWidth, +closeTL, -length-armLength - propShortLength - propWidth],     #4  100
                [-width-armLength + propShortLength + propWidth, +closeTU, -length-armLength - propShortLength + propWidth],     #5  101
                [-width-armLength + propShortLength + propWidth, +closeBU, -length-armLength - propShortLength + propWidth],     #6  102
                [-width-armLength + propShortLength - propWidth, +closeBL, -length-armLength - propShortLength - propWidth],     #7  103

                #Middle narrow tip
                [-width-armLength - propNarrowWidth, +endT, -length-armLength - propNarrowWidth],    #4  104
                [-width-armLength + propNarrowWidth, +endT, -length-armLength + propNarrowWidth],    #5  105
                [-width-armLength + propNarrowWidth, +endB, -length-armLength + propNarrowWidth],    #6  106
                [-width-armLength - propNarrowWidth, +endB, -length-armLength - propNarrowWidth],    #7  107

                #South West close wide
                [-width-armLength - propShortLength - propWidth, +closeTU, -length-armLength + propShortLength - propWidth],     #4  108
                [-width-armLength - propShortLength + propWidth, +closeTL, -length-armLength + propShortLength + propWidth],     #5  109
                [-width-armLength - propShortLength + propWidth, +closeBL, -length-armLength + propShortLength + propWidth],     #6  110
                [-width-armLength - propShortLength - propWidth, +closeBU, -length-armLength + propShortLength - propWidth],     #7  111
                #South West far wide
                [-width-armLength - propNarrowLength - propWidth, +farTU, -length-armLength + propNarrowLength - propWidth],     #4  112
                [-width-armLength - propNarrowLength + propWidth, +farTL, -length-armLength + propNarrowLength + propWidth],     #5  113
                [-width-armLength - propNarrowLength + propWidth, +farBL, -length-armLength + propNarrowLength + propWidth],     #6  114
                [-width-armLength - propNarrowLength - propWidth, +farBU, -length-armLength + propNarrowLength - propWidth],     #7  115
                #South West far narrow tip
                [-width-armLength - propLength - propNarrowWidth, +tipTU, -length-armLength + propLength - propNarrowWidth],     #0  116
                [-width-armLength - propLength + propNarrowWidth, +tipTL, -length-armLength + propLength + propNarrowWidth],     #1  117
                [-width-armLength - propLength + propNarrowWidth, +tipBL, -length-armLength + propLength + propNarrowWidth],     #2  118
                [-width-armLength - propLength - propNarrowWidth, +tipBU, -length-armLength + propLength - propNarrowWidth],     #3  119

                #creates prop 3 on arm 3

                #North West far narrow tip
                [+width+armLength - propLength - propNarrowWidth, +tipTU, -length-armLength - propLength + propNarrowWidth],     #0  120
                [+width+armLength - propLength + propNarrowWidth, +tipTL, -length-armLength - propLength - propNarrowWidth],     #1  121
                [+width+armLength - propLength + propNarrowWidth, +tipBL, -length-armLength - propLength - propNarrowWidth],     #2  122
                [+width+armLength - propLength - propNarrowWidth, +tipBU, -length-armLength - propLength + propNarrowWidth],     #3  123
                #North West far wide
                [+width+armLength - propNarrowLength - propWidth, +farTU, -length-armLength - propNarrowLength + propWidth],     #4  124
                [+width+armLength - propNarrowLength + propWidth, +farTL, -length-armLength - propNarrowLength - propWidth],     #5  125
                [+width+armLength - propNarrowLength + propWidth, +farBL, -length-armLength - propNarrowLength - propWidth],     #6  126
                [+width+armLength - propNarrowLength - propWidth, +farBU, -length-armLength - propNarrowLength + propWidth],     #7  127
                #North West close wide
                [+width+armLength - propShortLength - propWidth, +closeTU, -length-armLength - propShortLength + propWidth],     #4  128
                [+width+armLength - propShortLength + propWidth, +closeTL, -length-armLength - propShortLength - propWidth],     #5  129
                [+width+armLength - propShortLength + propWidth, +closeBL, -length-armLength - propShortLength - propWidth],     #6  130
                [+width+armLength - propShortLength - propWidth, +closeBU, -length-armLength - propShortLength + propWidth],     #7  131

                #Middle narrow tip
                [+width+armLength - propNarrowWidth, +endT, -length-armLength + propNarrowWidth],    #4  132
                [+width+armLength + propNarrowWidth, +endT, -length-armLength - propNarrowWidth],    #5  133
                [+width+armLength + propNarrowWidth, +endB, -length-armLength - propNarrowWidth],    #6  134
                [+width+armLength - propNarrowWidth, +endB, -length-armLength + propNarrowWidth],    #7  135

                #South East close wide
                [+width+armLength + propShortLength - propWidth, +closeTL, -length-armLength + propShortLength + propWidth],     #4  136
                [+width+armLength + propShortLength + propWidth, +closeTU, -length-armLength + propShortLength - propWidth],     #5  137
                [+width+armLength + propShortLength + propWidth, +closeBU, -length-armLength + propShortLength - propWidth],     #6  138
                [+width+armLength + propShortLength - propWidth, +closeBL, -length-armLength + propShortLength + propWidth],     #7  139
                #South East far wide
                [+width+armLength + propNarrowLength - propWidth, +farTL, -length-armLength + propNarrowLength + propWidth],     #4  140
                [+width+armLength + propNarrowLength + propWidth, +farTU, -length-armLength + propNarrowLength - propWidth],     #5  141
                [+width+armLength + propNarrowLength + propWidth, +farBU, -length-armLength + propNarrowLength - propWidth],     #6  142
                [+width+armLength + propNarrowLength - propWidth, +farBL, -length-armLength + propNarrowLength + propWidth],     #7  143
                #South East far narrow tip
                [+width+armLength + propLength - propNarrowWidth, +tipTL, -length-armLength + propLength + propNarrowWidth],     #0  144
                [+width+armLength + propLength + propNarrowWidth, +tipTU, -length-armLength + propLength - propNarrowWidth],     #1  145
                [+width+armLength + propLength + propNarrowWidth, +tipBU, -length-armLength + propLength - propNarrowWidth],     #2  146
                [+width+armLength + propLength - propNarrowWidth, +tipBL, -length-armLength + propLength + propNarrowWidth],     #3  147

                #creates prop 2 on arm 2

                #North West far narrow tip
                [-width-armLength - propLength - propNarrowWidth, +tipTU, +length+armLength - propLength + propNarrowWidth],     #0  148
                [-width-armLength - propLength + propNarrowWidth, +tipTL, +length+armLength - propLength - propNarrowWidth],     #1  149
                [-width-armLength - propLength + propNarrowWidth, +tipBL, +length+armLength - propLength - propNarrowWidth],     #2  150
                [-width-armLength - propLength - propNarrowWidth, +tipBU, +length+armLength - propLength + propNarrowWidth],     #3  151
                #North West far wide
                [-width-armLength - propNarrowLength - propWidth, +farTU, +length+armLength - propNarrowLength + propWidth],     #4  152
                [-width-armLength - propNarrowLength + propWidth, +farTL, +length+armLength - propNarrowLength - propWidth],     #5  153
                [-width-armLength - propNarrowLength + propWidth, +farBL, +length+armLength - propNarrowLength - propWidth],     #6  154
                [-width-armLength - propNarrowLength - propWidth, +farBU, +length+armLength - propNarrowLength + propWidth],     #7  155
                #North West close wide
                [-width-armLength - propShortLength - propWidth, +closeTU, +length+armLength - propShortLength + propWidth],     #4  156
                [-width-armLength - propShortLength + propWidth, +closeTL, +length+armLength - propShortLength - propWidth],     #5  157
                [-width-armLength - propShortLength + propWidth, +closeBL, +length+armLength - propShortLength - propWidth],     #6  158
                [-width-armLength - propShortLength - propWidth, +closeBU, +length+armLength - propShortLength + propWidth],     #7  159

                #Middle narrow tip
                [-width-armLength - propNarrowWidth, +endT, +length+armLength + propNarrowWidth],    #4  160
                [-width-armLength + propNarrowWidth, +endT, +length+armLength - propNarrowWidth],    #5  161
                [-width-armLength + propNarrowWidth, +endB, +length+armLength - propNarrowWidth],    #6  162
                [-width-armLength - propNarrowWidth, +endB, +length+armLength + propNarrowWidth],    #7  163

                #South East close wide
                [-width-armLength + propShortLength - propWidth, +closeTL, +length+armLength + propShortLength + propWidth],     #4  164
                [-width-armLength + propShortLength + propWidth, +closeTU, +length+armLength + propShortLength - propWidth],     #5  165
                [-width-armLength + propShortLength + propWidth, +closeBU, +length+armLength + propShortLength - propWidth],     #6  166
                [-width-armLength + propShortLength - propWidth, +closeBL, +length+armLength + propShortLength + propWidth],     #7  167
                #South East far wide
                [-width-armLength + propNarrowLength - propWidth, +farTL, +length+armLength + propNarrowLength + propWidth],     #4  168
                [-width-armLength + propNarrowLength + propWidth, +farTU, +length+armLength + propNarrowLength - propWidth],     #5  169
                [-width-armLength + propNarrowLength + propWidth, +farBU, +length+armLength + propNarrowLength - propWidth],     #6  170
                [-width-armLength + propNarrowLength - propWidth, +farBL, +length+armLength + propNarrowLength + propWidth],     #7  171
                #South East far narrow tip
                [-width-armLength + propLength - propNarrowWidth, +tipTL, +length+armLength + propLength + propNarrowWidth],     #0   172
                [-width-armLength + propLength + propNarrowWidth, +tipTU, +length+armLength + propLength - propNarrowWidth],     #1   173
                [-width-armLength + propLength + propNarrowWidth, +tipBU, +length+armLength + propLength - propNarrowWidth],     #2  174
                [-width-armLength + propLength - propNarrowWidth, +tipBL, +length+armLength + propLength + propNarrowWidth]      #3  175
                ])

        # Each face contains indices into points array above
        faces = [(50,49,48,51),(59,51,48,56),(58,59,56,57), #top of the Box
                 (40,41,42,43),(41,45,46,42),(45,44,47,46),(44,40,43,47),(40,44,45,41),(43,42,46,47),   #arrow Head
                 (32,33,34,35),(33,37,38,34),(37,36,39,38),(36,32,35,39),(32,36,37,33),(35,34,38,39),   #arrow Body
                 (54,55,52,53),(54,53,61,62),(62,61,60,63),(48,49,55,54),(49,50,52,55),(50,51,53,52),(51,59,61,53),(59,58,60,61),(58,57,63,60),(57,56,62,63),(56,48,54,62), #rest of the box
                 
                 (1,5,6,2),(5,4,7,6),(4,0,3,7),(0,4,5,1),(3,2,6,7),                     #arm 1
                 (9,13,14,10),(13,12,15,14),(12,8,11,15),(8,12,13,9),(11,10,14,15),     #arm 2
                 (16,17,18,19),(17,21,22,18),(20,16,19,23),(16,20,21,17),(19,18,22,23), #arm 3
                 (24,25,26,27),(25,29,30,26),(28,24,27,31),(24,28,29,25),(27,26,30,31), #arm 4
                 
                 (92,93,94,95),(93,97,98,94),(97,96,99,98),(96,92,95,99),(92,96,97,93),(95,94,98,99),(97,101,102,98),(101,100,103,102),(100,96,99,103),(96,100,101,97),(99,98,102,103),(101,105,106,102),(104,100,103,107),(100,104,105,101),(103,102,106,107),(105,109,110,106),(108,104,107,111),(104,108,109,105),(107,106,110,111),(109,113,114,110),(112,108,111,115),(108,112,113,109),(111,110,114,115),(113,117,118,114),(117,116,119,118),(116,112,115,119),(112,116,117,113),(115,114,118,119),   #prop 4
                 (120,121,122,123),(121,125,126,122),(124,120,123,127),(120,124,125,121),(123,122,126,127),(125,129,130,126),(128,124,127,131),(124,128,129,125),(127,126,130,131),(129,133,134,130),(132,128,131,135),(128,132,133,129),(131,130,134,135),(133,137,138,134),(136,132,135,139),(132,136,137,133),(135,134,138,139),(137,141,142,138),(140,136,139,143),(136,140,141,137),(139,138,142,143),(141,145,146,142),(145,144,147,146),(144,140,143,147),(140,144,145,141),(143,142,146,147),       #prop 3
                 (148,149,150,151),(149,153,154,150),(152,148,151,155),(148,152,153,149),(151,150,154,155),(153,157,158,154),(156,152,155,159),(152,156,157,153),(155,154,158,159),(157,161,162,158),(160,156,159,163),(156,160,161,157),(159,158,162,163),(161,165,166,162),(164,160,163,167),(160,164,165,161),(163,162,166,167),(165,169,170,166),(168,164,167,171),(164,168,169,165),(167,166,170,171),(169,173,174,170),(173,172,175,174),(172,168,171,175),(168,172,173,169),(171,170,174,175),       #prop 2
                 (64,65,66,67),(65,69,70,66),(68,64,67,71),(64,68,69,65),(67,66,70,71),(69,73,74,70),(72,68,71,75),(68,72,73,69),(71,70,74,75),(73,77,78,74),(76,72,75,79),(72,76,77,73),(75,74,78,79),(77,81,82,78),(80,76,79,83),(76,80,81,77),(79,78,82,83),(81,85,86,82),(84,80,83,87),(80,84,85,81),(83,82,86,87),(85,89,90,86),(89,88,91,90),(88,84,87,91),(84,88,89,85),(87,86,90,91),   #prop 1
                 ]

        lightGrey = '#72716d'
        grey = '#665f59'
        darkGrey ='#4c4641'
        darkRed = '#993838'
        red = 'red'
        green = '#31e224'
        darkGreen = '#2b7f24'
        
        colors = [
                  lightGrey,lightGrey,lightGrey,                        #box Top

                  'red','red','red','red','red','red',                  #arrow head
                  'red','red','red','red','red','red' ,                 #arrow body
                  
                  lightGrey,lightGrey,lightGrey,                        #box bottom
                  grey,grey,grey,                                       #box North
                  grey,                                                 #box East
                  grey,grey,grey,                                       #box South
                  grey,                                                 #box West

                  lightGrey, grey, lightGrey, darkGrey, darkGrey,       #arm 1
                  lightGrey, grey, lightGrey, darkGrey, darkGrey,       #arm 2
                  grey, lightGrey, lightGrey, darkGrey, darkGrey,       #arm 3
                  grey, lightGrey, lightGrey, darkGrey, darkGrey,       #arm 4

                  #prop 4
                  darkGreen,darkGreen,darkGreen,darkGreen,green,
                  green,darkGreen,darkGreen,darkGreen,
                  green,green,darkGreen,darkGreen,
                  green,green,darkGreen,darkGreen,
                  green,green,darkGreen,
                  darkGreen,green,green,darkGreen,
                  darkGreen,darkGreen,green,green,

                  #prop 3
                  darkGreen,darkGreen,darkGreen,green,
                  green,darkGreen,darkGreen,green,
                  green,darkGreen,darkGreen,green,
                  green,darkGreen,darkGreen,green,
                  green,darkGreen,darkGreen,green,
                  green,darkGreen,darkGreen,darkGreen,green,green,

                  #prop 2
                  darkRed,darkRed,darkRed,red,
                  red,darkRed,darkRed,red,
                  red,darkRed,darkRed,red,
                  red,darkRed,darkRed,red,
                  red,darkRed,darkRed,red,
                  red,darkRed,darkRed,darkRed,red,red,

                  #prop 1
                  darkRed,darkRed,darkRed,red,red,
                  darkRed,darkRed,red,red,
                  darkRed,darkRed,red,red,
                  darkRed,darkRed,red,
                  red,darkRed,darkRed,red,
                  red,darkRed,darkRed,darkRed,red,red,
                  ] 
        return points, faces, colors        
    

# Testing ==============================================================================================================

class MSPDriver(object):

    def __init__(self, root, canvas):

        self.root = root
        self.canvas = canvas

        #self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM )
        self.sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.sock.connect((BT_ADDR, BT_PORT))

        # MSPPG
        self.parser = msppg.MSP_Parser()
        self.parser.set_ATTITUDE_Handler(self._attitude_message_handler)
        self.request = msppg.serialize_ATTITUDE_Request()

        self.yaw, self.pitch, self.roll = 0, 0, 0
        
        thread = threading.Thread(target = self._read_fmu)
        thread.daemon = True
        thread.start()

        self._send_request()

    def _send_request(self):

        self.sock.send(self.request)
        

    def _read_fmu(self):

        while True:

            self.parser.parse(self.sock.recv(1))
                    
    def _attitude_message_handler(self, x, y, z):

        self.pitch = -y/10.
        self.roll = x/10.
        self.yaw = z

        self._send_request()

    def getYawPitchRoll(self):

        return self.yaw, self.pitch, self.roll
        

if __name__ == "__main__":

    width = 800
    height = 800

    root = tkinter.Tk()

    root.geometry('%dx%d+%d+%d' % (width, height, 200, 200))

    canvas = tkinter.Canvas(root, width=width, height=height, background='black')

    driver = MSPDriver(root, canvas)

    canvas.pack()

    Display(driver, simulation=True).start()

    tkinter.mainloop()

