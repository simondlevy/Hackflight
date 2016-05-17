#!/usr/bin/env python

# Class for testing motors ====================================================

MOTORS_IMAGE_FILE   = 'media/motors.gif'
MOTORS1_IMAGE_FILE  = 'media/motors1.gif'
MOTORS2_IMAGE_FILE  = 'media/motors2.gif'
MOTORS3_IMAGE_FILE  = 'media/motors3.gif'
MOTORS4_IMAGE_FILE  = 'media/motors4.gif'

MOTORS_IMAGE_SCALEDOWN = 3

MOTORS_IMAGE_X   = 40
MOTORS_IMAGE_Y   = 50

MOTOR_SCALE_X  = 300
MOTOR_SCALE_Y  = 90
MOTOR_SCALE_LENGTH = 200

MOTORS_WARNING_TEXT = 'I have removed the propellers and am ready to spin the motors safely.'

MOTORS_WARNING_X = 40
MOTORS_WARNING_Y = 350

MOTORS_LEFT_X   = 40
MOTORS_RIGHT_X  = 165
MOTORS_TOP_Y    = 60
MOTORS_BOTTOM_Y = 220
MOTORS_RADIUS   = 20

from Tkinter import *
from math import sqrt

from dialog import Dialog

class Motors(Dialog):

    def __init__(self, driver):

        Dialog.__init__(self, driver)

        # Add a quadcopter image for motor testing
        (self.image_motors,self.label_motors) = self._load_photo(MOTORS_IMAGE_FILE)
        (self.image_motors1,self.label_motors1) = self._load_photo(MOTORS1_IMAGE_FILE)
        (self.image_motors2,self.label_motors2) = self._load_photo(MOTORS2_IMAGE_FILE)
        (self.image_motors3,self.label_motors3) = self._load_photo(MOTORS3_IMAGE_FILE)
        (self.image_motors4,self.label_motors4) = self._load_photo(MOTORS4_IMAGE_FILE)

        # Add a warning checkbox for motor testing
        self.checkbox_var = IntVar()
        self.warning_motors = Checkbutton(self.driver.canvas, \
                variable=self.checkbox_var, command=self._checkbox_callback, \
                text=MOTORS_WARNING_TEXT, font=('Heletica', 14),  fg='red', bg='black', highlightthickness=0)

        # A a scale for motors
        self.scale = Scale(self.driver.canvas, from_=1850, to_=1000, command=self._scale_callback,
                orient=VERTICAL, length=MOTOR_SCALE_LENGTH, bg='black', fg='white')

        # Index of active motor (0 = none)
        self.active_motor = 0

    def start(self):

        self.warning_motors.place(x=MOTORS_WARNING_X, y=MOTORS_WARNING_Y)
        self._show_motors_image(self.label_motors)

    def stop(self):

        self.hide(self.warning_motors)
        self.hide(self.label_motors)
        self.hide(self.scale)
        self._hide_four_motors()

    def _show_motors_image(self, motors_label):

        motors_label.place(x=MOTORS_IMAGE_X, y=MOTORS_IMAGE_Y)

    def _scale_callback(self, valstr):

        self._send_motor_message(int(valstr))

    def _check_motor(self, event, x, y, label_motor, index):

        if sqrt((event.x-x)**2 + (event.y-y)**2) < MOTORS_RADIUS:

            self._hide_four_motors()

            self._show_motors_image(label_motor)

            # Reset scale to zero on new motor selection, for safety
            self.scale.set(0)

            # Power down previous motor if needed
            if self.active_motor > 0:

                self._send_motor_message(0)

            self.active_motor = index

    def _hide_four_motors(self):

        self.hide(self.label_motors1)
        self.hide(self.label_motors2)
        self.hide(self.label_motors3)
        self.hide(self.label_motors4)

    def _send_motor_message(self, value):

        self.driver.sendMotorMessage(self.active_motor, value)

    def _on_click(self, event):

        if self.checkbox_var.get():

            self._check_motor(event, MOTORS_LEFT_X,  MOTORS_TOP_Y,    self.label_motors4, 4)
            self._check_motor(event, MOTORS_RIGHT_X, MOTORS_TOP_Y,    self.label_motors2, 2)
            self._check_motor(event, MOTORS_RIGHT_X, MOTORS_BOTTOM_Y, self.label_motors1, 1)
            self._check_motor(event, MOTORS_LEFT_X,  MOTORS_BOTTOM_Y, self.label_motors3, 3)

    def _load_photo(self, filename):

        the_image = PhotoImage(file=filename).subsample(MOTORS_IMAGE_SCALEDOWN, MOTORS_IMAGE_SCALEDOWN)
        the_label = Label(self.driver.canvas, image=the_image, borderwidth=0)
        the_label.bind('<Button-1>', self._on_click)
        return the_image, the_label

    # Callback for motor-saftey checkbox
    def _checkbox_callback(self):

        # Checked
        if self.checkbox_var.get():

            # Hide no-motors image
            self.hide(self.label_motors)

            # Start with first motor
            self._show_motors_image(self.label_motors1)
            self.active_motor = 1

            # Reset the scale and show it
            self.scale.set('0')
            self.scale.place(x=MOTOR_SCALE_X, y=MOTOR_SCALE_Y)

        # Unchecked
        else:

            # Hide all motor images
            self.hide(self.label_motors1)
            self.hide(self.label_motors2)
            self.hide(self.label_motors3)
            self.hide(self.label_motors4)

            # Hide the scale
            self.hide(self.scale)

            # Show the no-motors image
            self._show_motors_image(self.label_motors)

            # Turn of any spinning motor
            if self.active_motor > 0:

                self._send_motor_message(0)

