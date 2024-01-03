'''
This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERECEIVERHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
'''


from dialog import Dialog
from debugging import debug

import tkinter as tk


class SensorsDialog(Dialog):

    UPDATE_MSEC = 1

    MOCAP_CTR_X = 190
    MOCAP_DOT_SIZE = 10
    MOCAP_MAXVAL = 50

    RANGER_PIXEL_COUNT = 16
    RANGER_CTR_X = 600
    RANGER_MAXVAL = 6000 # mm

    SQUARE_SIZE = 150
    SQUARE_CTR_Y = 220

    def __init__(self, viz):

        Dialog.__init__(self, viz)

    def start(self, delay_msec=UPDATE_MSEC):

        Dialog.start(self)

        self.canvas = self.viz.canvas

        self.mocap_dot = self.canvas.create_rectangle((0, 0, 0, 0),
                                                      fill='green')

        self._add_box(SensorsDialog.MOCAP_CTR_X, 'Motion Capture')

        self._add_box(SensorsDialog.RANGER_CTR_X, 'Ranging Camera')

        pixel_size = SensorsDialog.SQUARE_SIZE // 2

        ranger_corner_x = SensorsDialog.RANGER_CTR_X - SensorsDialog.SQUARE_SIZE
        ranger_corner_y = SensorsDialog.SQUARE_CTR_Y - SensorsDialog.SQUARE_SIZE

        pixpos = list(((
            ranger_corner_x + pixel_size * (k % 4),
            ranger_corner_y + pixel_size * (k // 4))
            for k in range(SensorsDialog.RANGER_PIXEL_COUNT)))

        self.ranger_pixels = [None for _ in range(SensorsDialog.RANGER_PIXEL_COUNT)]

        for k in range(SensorsDialog.RANGER_PIXEL_COUNT):
            self.ranger_pixels[k] = self.canvas.create_rectangle(
                        (pixpos[k][0],
                         pixpos[k][1],
                         pixpos[k][0]+pixel_size,
                         pixpos[k][1]+pixel_size), fill='gray')

        self.schedule_display_task(delay_msec)

    def _add_box(self, ctr_x, label):

        ctr_y = SensorsDialog.SQUARE_CTR_Y
        size = SensorsDialog.SQUARE_SIZE

        self.canvas.create_rectangle((ctr_x - size - 1,
                                      ctr_y - size - 1,
                                      ctr_x + size + 1,
                                      ctr_y + size + 1),
                                     outline='white')

        self.canvas.create_text(ctr_x - size + 70, ctr_y + size + 30,
                                anchor=tk.W, font=('Helvetica', 16),
                                fill='white', text=label)

    def _task(self):

        if self.running:

            # Display PAA3905 mocap -------------------------------------------

            mocap_dx, mocap_dy = self.viz.getMocap()

            mocap_dot_x = SensorsDialog._scale_mocap(mocap_dx) + SensorsDialog.MOCAP_CTR_X
            mocap_dot_y = SensorsDialog._scale_mocap(mocap_dy) + SensorsDialog.SQUARE_CTR_Y

            mocap_dot_size = SensorsDialog.MOCAP_DOT_SIZE // 2

            self.canvas.coords(self.mocap_dot,
                               (mocap_dot_x - mocap_dot_size,
                                mocap_dot_y - mocap_dot_size,
                                mocap_dot_x + mocap_dot_size,
                                mocap_dot_y + mocap_dot_size))

            # Display VL53l% ranging --------------------------------------------

            for k, val in enumerate(self.viz.getRanger()):
                scaled = int(val / SensorsDialog.RANGER_MAXVAL * 256)
                self.canvas.itemconfig(self.ranger_pixels[k],
                                       fill='#' + ('%02X' % scaled)*3)
            debug('')

            # Reschedule this display task
            self.schedule_display_task(SensorsDialog.UPDATE_MSEC)

    def _scale_mocap(val):

        return int(val / SensorsDialog.MOCAP_MAXVAL * SensorsDialog.SQUARE_SIZE)
