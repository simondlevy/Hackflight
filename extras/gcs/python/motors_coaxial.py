#!/usr/bin/env python
'''
Class for testing coaxial-layout motors in GCS

Copyright (C) Simon D. Levy 2021

MIT License
'''

import tkinter as tk
from dialog import Dialog
from motor_scale import MotorScale

WARNING_TEXT = \
        'I have removed the rotors and am ready to spin the motors safely.'

WARNING_X = 40
WARNING_Y = 350


class MotorsCoaxial(Dialog):

    def __init__(self, driver):

        Dialog.__init__(self, driver)

        # Add a warning checkbox for motor testing
        self.checkbox_var = tk.IntVar()
        self.warning = tk.Checkbutton(self.driver.canvas,
                                      variable=self.checkbox_var,
                                      command=self._checkbox_callback,
                                      text=WARNING_TEXT,
                                      font=('Heletica', 14), fg='red',
                                      bg='black', highlightthickness=0)

        # A a scale for motors
        self.servo1_scale = MotorScale(self)

        # Index of active motor (0 = none)
        self.active_motor = 0

    def start(self):

        Dialog.start(self)

        self.warning.place(x=WARNING_X, y=WARNING_Y)
        self._show_motors_image(self.label_motors)

    def stop(self):

        Dialog.stop(self)

        self.warning.deselect()
        self.hide(self.warning)
        self.servo1_scale.hide()
        self._turn_off_active()

    # Callback for motor-saftey checkbox
    def _checkbox_callback(self):

        # Checked
        if self.checkbox_var.get():

            # Start with first motor
            self.active_motor = 1

            # Reset the scale and show it
            self.servo1_scale.start()

        # Unchecked
        else:

            # Hide the scale
            self.servo1_scale.hide()

            # Turn of any spinning motor
            self._turn_off_active()

    def _turn_off_active(self):
        return
        # if self.driver.connected and self.active_motor > 0:
        #     self._send_motor_message(0)
