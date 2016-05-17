'''
dialog.py : generic dialog code for Hackflight GCS

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

class Dialog(object):

    def __init__(self, driver):

        self.driver = driver

        self.width = int(self.driver.canvas['width'])
        self.height = int(self.driver.canvas['height'])

        self.driver.root.bind("<Key>", self._check_quit)
        
    def hide(self, widget):

        self.driver.hide(widget)

    def schedule_display_task(self, delay_msec):

        self.driver.scheduleTask(delay_msec, self._task)

    def delete(self, widget):

        self.driver.canvas.delete(widget)

    def place(self, widget, x, y):

        widget.place(x=x, y=y, width=self.width, height=self.height)   

    def _check_quit(self, event):

        if ord(event.char) == 27: # ESC
            exit(0)


