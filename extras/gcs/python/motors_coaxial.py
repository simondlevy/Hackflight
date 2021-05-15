#!/usr/bin/env python
'''
Class for testing Multiwii quad-X layout motors in GCS

Copyright (C) Simon D. Levy 2021

MIT License
'''

import tkinter as tk
from dialog import Dialog

MOTOR_SCALE_X = 300
MOTOR_SCALE_Y = 90
MOTOR_SCALE_LENGTH = 200

MOTORS_WARNING_TEXT = \
        'I have removed the rotors and am ready to spin the motors safely.'

MOTORS_WARNING_X = 40
MOTORS_WARNING_Y = 350

MOTORS_LEFT_X = 40
MOTORS_RIGHT_X = 165
MOTORS_TOP_Y = 60
MOTORS_BOTTOM_Y = 220
MOTORS_RADIUS = 20


class MotorsCoaxial(Dialog):

    def __init__(self, driver):

        Dialog.__init__(self, driver)

        # Add a warning checkbox for motor testing
        self.checkbox_var = tk.IntVar()
        self.warning_motors = tk.Checkbutton(self.driver.canvas,
                                             variable=self.checkbox_var,
                                             command=self._checkbox_callback,
                                             text=MOTORS_WARNING_TEXT,
                                             font=('Heletica', 14), fg='red',
                                             bg='black', highlightthickness=0)

        # A a scale for motors
        self.scale = tk.Scale(self.driver.canvas, from_=100, to_=0,
                              command=self._scale_callback,
                              orient=tk.VERTICAL, length=MOTOR_SCALE_LENGTH,
                              bg='black', fg='white')

        # A label for the scale
        self.scale_label = tk.Label(self.driver.canvas, text='%', bg='black',
                                    fg='white')

        # Index of active motor (0 = none)
        self.active_motor = 0

    def start(self):

        Dialog.start(self)

        self.warning_motors.place(x=MOTORS_WARNING_X, y=MOTORS_WARNING_Y)
        self._show_motors_image(self.label_motors)

    def stop(self):

        Dialog.stop(self)

        self.warning_motors.deselect()
        self.hide(self.warning_motors)
        self.hide(self.scale)
        self.hide(self.scale_label)
        self._turn_off_active()

    def _scale_callback(self, valstr):

        self._send_motor_message(int(valstr))

    def _send_motor_message(self, percent):

        return
        self.driver.sendMotorMessage(self.active_motor, percent)

    # Callback for motor-saftey checkbox
    def _checkbox_callback(self):

        # Checked
        if self.checkbox_var.get():

            # Start with first motor
            self.active_motor = 1

            # Reset the scale and show it
            self.scale.set('0')
            self.scale.place(x=MOTOR_SCALE_X, y=MOTOR_SCALE_Y)
            self.scale_label.place(x=MOTOR_SCALE_X+20,
                                   y=MOTOR_SCALE_Y+MOTOR_SCALE_LENGTH+10)

        # Unchecked
        else:

            # Hide the scale
            self.hide(self.scale)
            self.hide(self.scale_label)

            # Turn of any spinning motor
            self._turn_off_active()

    def _turn_off_active(self):
        if self.driver.connected and self.active_motor > 0:
            self._send_motor_message(0)
