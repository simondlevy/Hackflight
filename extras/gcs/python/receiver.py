#!/usr/bin/env python
'''
receiver.py : class for displaying PWM values from receiver

Copyright (C) Simon D. Levy 2021
MIT License
'''

UPDATE_MSEC = 1

from dialog import Dialog

import tkcompat as tk

class Receiver(Dialog):

    def __init__(self, driver):

        Dialog.__init__(self, driver)

        self.running = False

    def start(self, delay_msec=UPDATE_MSEC):

        Dialog.start(self)

        self.throttle_gauge = self._new_gauge(0, 'Throttle', 'red', minval=0)   
        self.roll_gauge     = self._new_gauge(1, '    Roll', 'blue')           
        self.pitch_gauge    = self._new_gauge(2, '   Pitch', 'green')         
        self.yaw_gauge      = self._new_gauge(3, '     Yaw', 'orange')       
        self.aux1_gauge     = self._new_gauge(4, '    Aux1', 'purple')
        self.aux2_gauge     = self._new_gauge(5, '    Aux2', 'yellow')

        self.schedule_display_task(delay_msec)

    def stop(self):

        Dialog.stop(self)

    def _task(self):

        if self.running:

            channels = self.driver.getChannels()

            self.throttle_gauge.update(channels[0]) # Throttle
            self.roll_gauge.update    (channels[1]) # Roll
            self.pitch_gauge.update   (channels[2]) # Pitch
            self.yaw_gauge.update     (channels[3]) # Yaw
            self.aux1_gauge.update    (channels[4]) # Aux1
            self.aux2_gauge.update    (channels[5]) # Aux2

            self.schedule_display_task(UPDATE_MSEC)

            # Add a label for arming if needed
            self.driver.checkArmed() 

    def _new_gauge(self, offset, name, color, minval=-1):

        return HorizontalGauge(self, 100, 60+85*offset, 600, 40, color, name, minval, +1, '%0.2f')

class HorizontalGauge(object):

    def __init__(self, owner, left, bottom, width, height, color, label, minval, maxval, fmt):

        self.owner = owner

        right = left + width

        self.width  = width
        self.minval = minval
        self.maxval = maxval

        top    = bottom - height
        bbox = (left, bottom, right, top)
        self.bbox = bbox
        self.rect = self.owner.driver.canvas.create_rectangle(bbox, fill=color)
        self.owner.driver.canvas.create_rectangle((bbox[0]-1, bbox[1]-1, bbox[2]+1, bbox[3]+1), outline='white')

        self._create_label(left-70, (top+bottom)/2, text=label)

        y = bottom + 10
        self._create_label(left-10,  y, text = fmt % minval)
        self._create_label(right-30, y, text = fmt % maxval)

        self.label = self._create_label((left+right)/2-25, top+height/2)

    def update(self, newval):

        new_width = self.width * (newval-self.minval) / (self.maxval - self.minval)
        bbox = self.bbox
        self.owner.driver.canvas.coords(self.rect, (bbox[0], bbox[1], bbox[0]+new_width, bbox[3]))

        self.owner.driver.canvas.itemconfigure(self.label, text=('%0.2f' % newval))

    def _create_label(self, x, y, text=''):

        return self.owner.driver.canvas.create_text(x, y, anchor=tk.W, font=('Helvetica', 12), fill='white', text=text)



