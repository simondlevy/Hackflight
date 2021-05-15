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

SERVO1_X = 150
SERVO2_X = 300
MOTOR1_X = 450
MOTOR2_X = 600

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

        # Add scales for servos, motors
        self.servo1_scale = MotorScale(self, SERVO1_X, 'Servo 1', -50, +50)
        self.servo2_scale = MotorScale(self, SERVO2_X, 'Servo 2', -50, +50)
        self.motor1_scale = MotorScale(self, MOTOR1_X, 'Motor 1', 0, 100)
        self.motor2_scale = MotorScale(self, MOTOR2_X, 'Motor 2', 0, 100)

        # Index of active motor (0 = none)
        self.active_motor = 0

    def start(self):

        Dialog.start(self)

        self.warning.place(x=WARNING_X, y=WARNING_Y)

        self.servo1_scale.start()
        self.servo2_scale.start()

    def stop(self):

        Dialog.stop(self)

        self.warning.deselect()
        self.hide(self.warning)
        self.servo1_scale.hide()
        self.servo2_scale.hide()
        self.motor1_scale.hide()
        self.motor2_scale.hide()
        self._turn_off_active()

    # Callback for motor-saftey checkbox
    def _checkbox_callback(self):

        # Checked
        if self.checkbox_var.get():

            # Start with first motor
            self.active_motor = 1

            # Reset the scales and show them
            self.servo1_scale.start()
            self.servo2_scale.start()
            self.motor1_scale.start()
            self.motor2_scale.start()

        # Unchecked
        else:

            # Turn of any spinning motor
            self._turn_off_active()

            # Hide motors
            self.motor1_scale.hide()
            self.motor2_scale.hide()

    def _turn_off_active(self):
        return
        # if self.driver.connected and self.active_motor > 0:
        #     self._send_motor_message(0)
