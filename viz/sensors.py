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

import tkinter as tk


class Sensors(Dialog):

    UPDATE_MSEC = 1

    RANGER_PIXEL_COUNT = 16

    MOCAP_CTR_X = 190
    MOCAP_DOT_SIZE = 10

    RANGER_CTR_X = 600

    SQUARE_SIZE = 150
    SQUARE_CTR_Y = 220

    def __init__(self, viz):

        Dialog.__init__(self, viz)

    def start(self, delay_msec=UPDATE_MSEC):

        Dialog.start(self)

        self.canvas = self.viz.canvas

        self.mocap_dot = self.canvas.create_rectangle((0, 0, 0, 0),
                                                      fill='green')

        self._add_box(Sensors.MOCAP_CTR_X, 'Motion Capture')

        self._add_box(Sensors.RANGER_CTR_X, 'Ranging Camera')

        pixel_size = Sensors.SQUARE_SIZE // 2

        ranger_corner_x = Sensors.RANGER_CTR_X - Sensors.SQUARE_SIZE
        ranger_corner_y = Sensors.SQUARE_CTR_Y - Sensors.SQUARE_SIZE

        pixpos = list(((
            ranger_corner_x + pixel_size * (k % 4),
            ranger_corner_y + pixel_size * (k // 4))
            for k in range(Sensors.RANGER_PIXEL_COUNT)))

        self.ranger_pixels = [None for _ in range(Sensors.RANGER_PIXEL_COUNT)]

        for k in range(Sensors.RANGER_PIXEL_COUNT):
            self.ranger_pixels[k] = self.canvas.create_rectangle(
                        (pixpos[k][0],
                         pixpos[k][1],
                         pixpos[k][0]+pixel_size,
                         pixpos[k][1]+pixel_size), fill='gray')

        self.schedule_display_task(delay_msec)

    def _add_box(self, ctr_x, label):

        ctr_y = Sensors.SQUARE_CTR_Y
        size = Sensors.SQUARE_SIZE

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

            for k, val in enumerate(self.viz.getRanger()):
                self.canvas.itemconfig(self.ranger_pixels[k],
                                       fill='#' + ('%02X' % val)*3)

            mocap_dx, mocap_dy = self.viz.getMocap()

            mocap_dot_x = mocap_dx + Sensors.MOCAP_CTR_X
            mocap_dot_y = mocap_dy + Sensors.SQUARE_CTR_Y

            size = Sensors.MOCAP_DOT_SIZE // 2

            self.canvas.coords(self.mocap_dot,
                               (mocap_dot_x - size,
                                mocap_dot_y - size,
                                mocap_dot_x + size,
                                mocap_dot_y + size))

            self.schedule_display_task(Sensors.UPDATE_MSEC)
