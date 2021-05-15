#!/usr/bin/env python
'''
Class for sliding scales

Copyright (C) Simon D. Levy 2021

MIT License
'''

import tkinter as tk

MOTOR_SCALE_X = 300
MOTOR_SCALE_Y = 90
MOTOR_SCALE_LENGTH = 200


class MotorScale(object):

    def __init__(self, canvas):

        # A a scale for motors
        self.scale = tk.Scale(canvas, from_=100, to_=0,
                              command=self.callback,
                              orient=tk.VERTICAL, length=MOTOR_SCALE_LENGTH,
                              bg='black', fg='white')

        # A label for the scale
        self.label = tk.Label(canvas, text='%', bg='black', fg='white')

    def hide(self):

        self.hide(self.scale)
        self.hide(self.scale_label)

    def callback(self, valstr):

        print(valstr)
