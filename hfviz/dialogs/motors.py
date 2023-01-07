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


import tkinter as tk
from math import sqrt
from dialog import Dialog
from resources import resource_path

from debugging import debug

class MotorScale(object):

    def __init__(self, dialog, x,
                 index=0, label='Motor', minval=0, maxval=100,
                 y=90, length=200):

        self.dialog = dialog
        self.x = x
        self.index = index
        self.y = y
        self.length = length

        canvas = dialog.viz.canvas

        # A a scale for motors
        self.scale = tk.Scale(canvas, from_=maxval, to_=minval,
                              command=self.callback,
                              orient=tk.VERTICAL, length=length,
                              bg='black', fg='white')

        # A label for the scale
        self.label = tk.Label(canvas, text=(label), bg='black', fg='white')

    def start(self):

        self.scale.set('0')
        self.scale.place(x=self.x, y=self.y)
        self.label.place(x=self.x, y=self.y+self.length+10)

    def hide(self):

        self.dialog.hide(self.scale)
        self.dialog.hide(self.label)

    def stop(self):

        self.hide()
        self.callback('0')  # ensures current motor turned off on exit

    def callback(self, valstr):

        if self.dialog.viz is not None: # Guard at startup
            motors = [1000] * 4
            motors[self.index-1] = int(1000 * (1 +  float(valstr)/100))
            self.dialog.viz.sendMotorMessage(motors)

    def setValue(self, value):

        self.scale.set(str(value))

    def setIndex(self, index):

        self.index = index

    def getIndex(self):

        return self.index

    def adjust(self, value):

        return value


class ServoScale(MotorScale):

    def __init__(self, dialog, x, index, label):

        MotorScale.__init__(self, dialog, x, index, label, -45, +45)


class MotorsDialog(Dialog):

    WARNING_TEXT = ('I have removed the rotors and am ready to spin the ' +
                    'motors safely.')
    WARNING_X = 40
    WARNING_Y = 350

    def __init__(self, viz):

        Dialog.__init__(self, viz)

        # Add a warning checkbox for motor testing
        self.checkbox_var = tk.IntVar()
        self.warning = tk.Checkbutton(self.viz.canvas,
                                      variable=self.checkbox_var,
                                      command=self._checkbox_callback,
                                      text=self.WARNING_TEXT,
                                      font=('Helvetica', 14), fg='red',
                                      bg='black', highlightthickness=0)

    def start(self):

        Dialog.start(self)
        self.warning.place(x=self.WARNING_X, y=self.WARNING_Y)

    def stop(self):

        Dialog.stop(self)

        self.warning.deselect()
        self.hide(self.warning)


class MotorsQuadXmwDialog(MotorsDialog):

    MOTORS_IMAGE_FILE = resource_path('motors.gif')
    MOTORS1_IMAGE_FILE = resource_path('motors1.gif')
    MOTORS2_IMAGE_FILE = resource_path('motors2.gif')
    MOTORS3_IMAGE_FILE = resource_path('motors3.gif')
    MOTORS4_IMAGE_FILE = resource_path('motors4.gif')

    MOTORS_IMAGE_SCALEDOWN = 3

    MOTORS_IMAGE_X = 40
    MOTORS_IMAGE_Y = 50

    MOTOR_SCALE_X = 300

    MOTORS_LEFT_X = 40
    MOTORS_RIGHT_X = 165
    MOTORS_TOP_Y = 60
    MOTORS_BOTTOM_Y = 220
    MOTORS_RADIUS = 20

    def __init__(self, viz):

        MotorsDialog.__init__(self, viz)

        # Add a quadcopter image for motor testing
        self.image_motors, self.label_motors = \
            self._load_photo(self.MOTORS_IMAGE_FILE)
        self.image_motors1, self.label_motors1 = \
            self._load_photo(self.MOTORS1_IMAGE_FILE)
        self.image_motors2, self.label_motors2 = \
            self._load_photo(self.MOTORS2_IMAGE_FILE)
        self.image_motors3, self.label_motors3 = \
            self._load_photo(self.MOTORS3_IMAGE_FILE)
        self.image_motors4, self.label_motors4 = \
            self._load_photo(self.MOTORS4_IMAGE_FILE)

        # A a scale for motors
        self.scale = MotorScale(self, self.MOTOR_SCALE_X)

        # A label for the scale
        self.scale_label = tk.Label(self.viz.canvas, text='%', bg='black',
                                    fg='white')

    def start(self):

        MotorsDialog.start(self)

        self._show_motors_image(self.label_motors)

    def stop(self):

        MotorsDialog.stop(self)

        self.hide(self.label_motors)
        self.scale.stop()
        self.hide(self.scale_label)
        self._hide_four_motors()
        self._turn_off_active()

    def _show_motors_image(self, motors_label):

        motors_label.place(x=self.MOTORS_IMAGE_X, y=self.MOTORS_IMAGE_Y)

    def _check_motor(self, event, x, y, label_motor, index):

        if sqrt((event.x-x)**2 + (event.y-y)**2) < self.MOTORS_RADIUS:

            self._hide_four_motors()

            self._show_motors_image(label_motor)

            # Reset scale to zero on new motor selection, for safety
            self.scale.setValue(0)
            self.scale.setIndex(index)

    def _hide_four_motors(self):

        self.hide(self.label_motors1)
        self.hide(self.label_motors2)
        self.hide(self.label_motors3)
        self.hide(self.label_motors4)

    def _on_click(self, event):

        if self.checkbox_var.get():

            self._check_motor(event, self.MOTORS_LEFT_X,  self.MOTORS_TOP_Y,
                              self.label_motors4, 4)
            self._check_motor(event, self.MOTORS_RIGHT_X, self.MOTORS_TOP_Y,
                              self.label_motors2, 2)
            self._check_motor(event, self.MOTORS_RIGHT_X, self.MOTORS_BOTTOM_Y,
                              self.label_motors1, 1)
            self._check_motor(event, self.MOTORS_LEFT_X,  self.MOTORS_BOTTOM_Y,
                              self.label_motors3, 3)

    def _load_photo(self, filename):

        the_file = tk.PhotoImage(file=filename)
        the_image = the_file.subsample(self.MOTORS_IMAGE_SCALEDOWN,
                                       self.MOTORS_IMAGE_SCALEDOWN)
        the_label = tk.Label(self.viz.canvas,
                             image=the_image, borderwidth=0)
        the_label.bind('<Button-1>', self._on_click)
        return the_image, the_label

    # Callback for motor-saftey checkbox
    def _checkbox_callback(self):

        # Checked
        if self.checkbox_var.get():

            self.scale.setIndex(1)

            # Hide no-motors image
            self.hide(self.label_motors)

            # Start with first motor
            self._show_motors_image(self.label_motors1)

            # Reset the scale and show it
            self.scale.start()

        # Unchecked
        else:

            # Hide all motor images
            self.hide(self.label_motors1)
            self.hide(self.label_motors2)
            self.hide(self.label_motors3)
            self.hide(self.label_motors4)

            # Hide the scale
            self.scale.stop()

            # Show the no-motors image
            self._show_motors_image(self.label_motors)

            # Turn of any spinning motor
            self._turn_off_active()

    def _turn_off_active(self):
        if self.viz.connected and self.scale.getIndex() > 0:
            self.scale.setIndex(0)


class MotorsCoaxialDialog(MotorsDialog):

    SERVO1_X = 150
    SERVO2_X = 300
    MOTOR1_X = 450
    MOTOR2_X = 600

    def __init__(self, viz):

        MotorsDialog.__init__(self, viz)

        # Add scales for servos, motors
        self.motor1_scale = MotorScale(self, self.MOTOR1_X, 1, 'Motor 1')
        self.motor2_scale = MotorScale(self, self.MOTOR2_X, 2, 'Motor 2')
        self.servo1_scale = ServoScale(self, self.SERVO1_X, 3, 'Servo 1')
        self.servo2_scale = ServoScale(self, self.SERVO2_X, 4, 'Servo 2')

    def start(self):

        MotorsDialog.start(self)

        self.servo1_scale.start()
        self.servo2_scale.start()

    def stop(self):

        MotorsDialog.stop(self)

        self.servo1_scale.hide()
        self.servo2_scale.hide()
        self.motor1_scale.hide()
        self.motor2_scale.hide()
        self._cut_motors()

    # Callback for motor-saftey checkbox
    def _checkbox_callback(self):

        # Checked
        if self.checkbox_var.get():

            # Reset the scales and show them
            self.servo1_scale.start()
            self.servo2_scale.start()
            self.motor1_scale.start()
            self.motor2_scale.start()

        # Unchecked
        else:

            # Turn of any spinning motor
            self._cut_motors()

            # Hide motors
            self.motor1_scale.stop()
            self.motor2_scale.stop()

    def _cut_motors(self):
        try:
            self.viz.sendMotorMessage(1, 0)
            self.viz.sendMotorMessage(2, 0)
            self.viz.sendMotorMessage(3, 0)
            self.viz.sendMotorMessage(4, 0)
        except Exception:
            return
