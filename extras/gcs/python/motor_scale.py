#!/usr/bin/env python
'''
Class for sliding scales for running motors

Copyright (C) Simon D. Levy 2021

MIT License
'''

import tkinter as tk

SCALE_Y = 90
SCALE_LENGTH = 200


class MotorScale(object):

    def __init__(self, dialog, x, index, label, minval=0, maxval=100):

        self.dialog = dialog
        self.x = x
        self.index = index

        canvas = dialog.gcs.canvas

        # A a scale for motors
        self.scale = tk.Scale(canvas, from_=maxval, to_=minval,
                              command=self.callback,
                              orient=tk.VERTICAL, length=SCALE_LENGTH,
                              bg='black', fg='white')

        # A label for the scale
        self.label = tk.Label(canvas, text=(label), bg='black', fg='white')

    def start(self):

        self.scale.set('0')
        self.scale.place(x=self.x, y=SCALE_Y)
        self.label.place(x=self.x, y=SCALE_Y+SCALE_LENGTH+10)

    def hide(self):

        self.dialog.hide(self.scale)
        self.dialog.hide(self.label)

    def callback(self, valstr):

        self.dialog.gcs.sendMotorMessage(self.index, int(valstr))

    def adjust(self, value):

        return value


class ServoScale(MotorScale):

    def __init__(self, dialog, x, index, label):

        MotorScale.__init__(self, dialog, x, index, label, -50, +50)
