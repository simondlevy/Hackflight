#!/usr/bin/env python
'''
Class for sliding scales

Copyright (C) Simon D. Levy 2021

MIT License
'''

import tkinter as tk

SCALE_X = 300
SCALE_Y = 90
SCALE_LENGTH = 200


class MotorScale(object):

    def __init__(self, dialog):

        self.dialog = dialog
        canvas = dialog.driver.canvas

        # A a scale for motors
        self.scale = tk.Scale(canvas, from_=100, to_=0,
                              command=self.callback,
                              orient=tk.VERTICAL, length=SCALE_LENGTH,
                              bg='black', fg='white')

        # A label for the scale
        self.label = tk.Label(canvas, text='%', bg='black', fg='white')

    def start(self):

        self.scale.set('0')
        self.scale.place(x=SCALE_X, y=SCALE_Y)
        self.label.place(x=SCALE_X+20, y=SCALE_Y+SCALE_LENGTH+10)

    def hide(self):

        self.dialog.hide(self.scale)
        self.dialog.hide(self.label)

    def callback(self, valstr):

        print(valstr)
