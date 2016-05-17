#!/usr/bin/env python

'''
messages.py : message-display class

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

from Tkinter import *

from dialog import Dialog

class Messages(Dialog):

    def __init__(self, driver):

        Dialog.__init__(self, driver)

        self.running = False

        self.message_count = 0
        self.current_message = None
        self.previous_message = None

        self.scrollbar = Scrollbar(driver.canvas)
        self.listbox = Listbox(driver.canvas, bg='black', fg='white')

        self.checkbox_var = IntVar()
        self.checkbox = Checkbutton(self.driver.canvas, 
                text='Autoscroll', selectcolor='black', variable=self.checkbox_var, 
                bg='black', fg='white', highlightthickness=0)
        self.checkbox.select()
        
        self.listbox.config(yscrollcommand=self.scrollbar.set)
        self.scrollbar.config(command=self.listbox.yview)

    def start(self, delay_msec=UPDATE_MSEC):

        self.schedule_display_task(delay_msec)

        self.running = True

        height = int(self.driver.canvas['height'])-100
        self.scrollbar.place(x=0, y=0, width=SCROLLBAR_WIDTH, height=height)
        self.listbox.place(x=SCROLLBAR_WIDTH, y=0, width=self.driver.canvas['width'], height=str(height))
        self.checkbox.place(x=10, y = height+10)

    def stop(self):

        self.hide(self.listbox)
        self.hide(self.scrollbar)
        self.hide(self.checkbox)

        self.running = False

    def setCurrentMessage(self, message):

        self.message_count += 1
        self.current_message = '%05d: %s' % (self.message_count, message)

    def _task(self):

        if self.running:


            if self.current_message != self.previous_message:
                self.listbox.insert(END, self.current_message)
                if self.checkbox_var.get():
                    self.listbox.see(END)

            self.previous_message = self.current_message

            self.schedule_display_task(UPDATE_MSEC)

            # Add a label for arming if needed
            self.driver.checkArmed() 
