#!/usr/bin/env python
'''
Message-display class for GCS

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

UPDATE_MSEC = 50
SCROLLBAR_WIDTH = 20

import tkcompat as tk

from dialog import Dialog

class Messages(Dialog):

    def __init__(self, driver):

        Dialog.__init__(self, driver)

        self.message_count = 0
        self.current_message = None
        self.previous_message = None

        self.scrollbar = tk.Scrollbar(driver.canvas)
        self.listbox = tk.Listbox(driver.canvas, bg='black', fg='white')

        self.checkbox_var = tk.IntVar()
        self.checkbox = tk.Checkbutton(self.driver.canvas, 
                text='Autoscroll', selectcolor='black', variable=self.checkbox_var, 
                bg='black', fg='white', highlightthickness=0)
        self.checkbox.select()
        
        self.listbox.config(yscrollcommand=self.scrollbar.set)
        self.scrollbar.config(command=self.listbox.yview)

    def start(self, delay_msec=UPDATE_MSEC):

        Dialog.start(self)

        self.schedule_display_task(delay_msec)

        height = int(self.driver.canvas['height'])-100
        self.scrollbar.place(x=0, y=0, width=SCROLLBAR_WIDTH, height=height)
        self.listbox.place(x=SCROLLBAR_WIDTH, y=0, width=self.driver.canvas['width'], height=str(height))
        self.checkbox.place(x=10, y = height+10)

    def stop(self):

        self.hide(self.listbox)
        self.hide(self.scrollbar)
        self.hide(self.checkbox)

    def setCurrentMessage(self, message):

        self.message_count += 1
        self.current_message = '%05d: %s' % (self.message_count, message)

    def _task(self):

        if self.running:


            if self.current_message != self.previous_message:
                self.listbox.insert(tk.END, self.current_message)
                if self.checkbox_var.get():
                    self.listbox.see(tk.END)

            self.previous_message = self.current_message

            self.schedule_display_task(UPDATE_MSEC)

            # Add a label for arming if needed
            self.driver.checkArmed() 
