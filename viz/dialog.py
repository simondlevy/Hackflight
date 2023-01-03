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


class Dialog(object):

    def __init__(self, viz):

        self.running = False

        self.viz = viz

        self.width = int(self.viz.canvas['width'])
        self.height = int(self.viz.canvas['height'])

        self.viz.root.bind("<Key>", self._check_quit)

    def hide(self, widget):

        self.viz.hide(widget)

    def schedule_display_task(self, delay_msec):

        self.viz.scheduleTask(delay_msec, self._task)

    def delete(self, widget):

        self.viz.canvas.delete(widget)

    def place(self, widget, x, y):

        widget.place(x=x, y=y, width=self.width, height=self.height)

    def start(self):

        self.running = True

    def stop(self):

        self.running = False

    def _check_quit(self, event):

        if ord(event.char) == 27:  # ESC
            exit(0)
