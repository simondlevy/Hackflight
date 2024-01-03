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


from debugging import debug
from dialog import Dialog
import tkinter as tk

UPDATE_MSEC = 1


class ReceiverDialog(Dialog):

    def __init__(self, viz):

        Dialog.__init__(self, viz)

        self.running = False

    def start(self, delay_msec=UPDATE_MSEC):

        Dialog.start(self)

        self.throttle_gauge = self._new_gauge(0, 'Throttle', 'red', minval=0)
        self.roll_gauge = self._new_gauge(1, '    Roll', 'blue')
        self.pitch_gauge = self._new_gauge(2, '   Pitch', 'green')
        self.yaw_gauge = self._new_gauge(3, '     Yaw', 'orange')
        self.aux1_gauge = self._new_gauge(4, '    Aux1', 'purple')
        self.aux2_gauge = self._new_gauge(5, '    Aux2', 'yellow')

        self.schedule_display_task(delay_msec)

    def stop(self):

        Dialog.stop(self)

    def _task(self):

        if self.running:

            channels = self.viz.getChannels()

            self.throttle_gauge.update(channels[0])  # Throttle
            self.roll_gauge.update(channels[1])      # Roll
            self.pitch_gauge.update(channels[2])     # Pitch
            self.yaw_gauge.update(channels[3])       # Yaw
            self.aux1_gauge.update(channels[4])      # Aux1
            self.aux2_gauge.update(channels[5])      # Aux2

            self.schedule_display_task(UPDATE_MSEC)

    def _new_gauge(self, offset, name, color, minval=-1):

        return HorizontalGauge(self, 100, 60+85*offset, 600, 40, color, name,
                               minval, +1, '%0.2f')


class HorizontalGauge(object):

    def __init__(self, owner, left, bottom, width, height, color, label,
                 minval, maxval, fmt):

        self.owner = owner

        right = left + width

        self.width = width
        self.minval = minval
        self.maxval = maxval

        top = bottom - height
        bbox = (left, bottom, right, top)
        self.bbox = bbox
        self.rect = self.owner.viz.canvas.create_rectangle(bbox, fill=color)
        self.owner.viz.canvas.create_rectangle((bbox[0]-1, bbox[1]-1,
                                               bbox[2]+1, bbox[3]+1),
                                               outline='white')

        self._create_label(left-70, (top+bottom)/2, text=label)

        y = bottom + 10
        self._create_label(left-10,  y, text=fmt % minval)
        self._create_label(right-30, y, text=fmt % maxval)

        self.label = self._create_label((left+right)/2-25, top+height/2)

    def update(self, newval):

        new_width = self.width * (newval-self.minval) / (self.maxval -
                                                         self.minval)
        bbox = self.bbox
        self.owner.viz.canvas.coords(self.rect,
                                     (bbox[0],
                                      bbox[1],
                                      bbox[0]+new_width,
                                      bbox[3]))

        self.owner.viz.canvas.itemconfigure(self.label,
                                            text=('%0.2f' % newval))

    def _create_label(self, x, y, text=''):

        return self.owner.viz.canvas.create_text(x, y, anchor=tk.W,
                                                 font=('Helvetica', 12),
                                                 fill='white', text=text)
