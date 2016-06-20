#!/usr/bin/env python
'''
receiver.py : class for displaying PWM values from receiver

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

UPDATE_MSEC = 1

PWM_MIN = 988
PWM_MAX = 2011

from Tkinter import *

from dialog import Dialog

class Receiver(Dialog):

    def __init__(self, driver):

        Dialog.__init__(self, driver)

        self.running = False

    def start(self, delay_msec=UPDATE_MSEC):

        self.roll_gauge     = self._new_gauge(0, '    Roll', 'blue')
        self.pitch_gauge    = self._new_gauge(1, '   Pitch', 'green')
        self.yaw_gauge      = self._new_gauge(2, '     Yaw', 'orange')
        self.throttle_gauge = self._new_gauge(3, 'Throttle', 'red')
        self.switch_gauge   = self._new_gauge(4, '     Aux', 'purple')

        self.schedule_display_task(delay_msec)

        self.running = True

    def stop(self):

        self.running = False

    def _task(self):

        if self.running:

            channels = self.driver.getChannels()

            self.roll_gauge.update    (channels[0])
            self.pitch_gauge.update   (channels[1])
            self.throttle_gauge.update(channels[3])
            self.yaw_gauge.update     (channels[2])
            self.switch_gauge.update  (channels[4])

            self.schedule_display_task(UPDATE_MSEC)

            # Add a label for arming if needed
            self.driver.checkArmed() 

    def _new_gauge(self, offset, name, color):

        return HorizontalGauge(self, 100, 60+85*offset, 600, 40, color, name, PWM_MIN, PWM_MAX, '%4d')

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

        self.owner.driver.canvas.itemconfigure(self.label, text=('%4d' % newval))

    def _create_label(self, x, y, text=''):

        return self.owner.driver.canvas.create_text(x, y, anchor=W, font=('Helvetica', 12), fill='white', text=text)



# Testing ==============================================================================================================

DISPLAY_WIDTH  = 800
DISPLAY_HEIGHT = 450

class ReceiverDriver(object):

    def __init__(self, root, canvas):

        self.root = root
        self.canvas = canvas

        self.index = 0
        self.value = PWM_MIN
        self.sign = +1

    def getChannels(self):

        self.value += self.sign

        if self.value == PWM_MAX:
            self.sign = -1

        if self.value == PWM_MIN:
            self.sign = +1
            self.index = (self.index + 1) % 5

        # fake up discrete switch
        value = self._discretize(self.value) if self.index == 4 else self.value

        retvals = [PWM_MIN]*5

        retvals[self.index] = value

        return retvals

    def _discretize(self, value):

        scaled = (float(value) - PWM_MIN) / (PWM_MAX - PWM_MIN)

        return PWM_MIN if scaled < 0.33 else ((PWM_MIN+PWM_MAX)/2 if scaled < .67 else PWM_MAX)

    def checkArmed(self):

        None

if __name__ == "__main__":

    root = Tk()

    root.geometry('%dx%d+%d+%d' % (DISPLAY_WIDTH, DISPLAY_HEIGHT, 200, 200))
    root.title('Receiver')

    canvas = Canvas(root, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, background='black')

    canvas.pack()

    driver = ReceiverDriver(root, canvas)

    receiver = Receiver(driver)

    receiver.start()

    mainloop()
