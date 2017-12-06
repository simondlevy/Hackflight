#!/usr/bin/env python

'''
hackflight.py : starter script for  Hackflight GCS

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

BAUD = 115200

DISPLAY_WIDTH  = 800
DISPLAY_HEIGHT = 500

BACKGROUND_COLOR = 'white'

CONNECTION_DELAY_MSEC  = 1000
BOARD_REPLY_DELAY_MSEC = 1000

USB_UPDATE_MSEC = 200

# These should agree with the values in firmware Config.PwmConfg
PWM_MIN = 1000
PWM_MAX = 2000

from serial import Serial
from serial.tools.list_ports import comports
from threading import Thread
import os

from msppg import *

from tkcompat import *

from imu import IMU
from motors import Motors
from receiver import Receiver
#from maps import Maps
#from messages import Messages

# GCS class runs the show =========================================================================================

class GCS:

    def __init__(self):

        # No communications or arming yet
        self.comms = None
        self.armed = False
        self.gotimu = False

        # Do basic Tk initialization
        self.root = Tk()
        self.root.configure(bg=BACKGROUND_COLOR)
        self.root.resizable(False, False)
        self.root.title('Hackflight Ground Control Station')
        left = (self.root.winfo_screenwidth() - DISPLAY_WIDTH) / 2
        top = (self.root.winfo_screenheight() - DISPLAY_HEIGHT) / 2
        self.root.geometry('%dx%d+%d+%d' % (DISPLAY_WIDTH, DISPLAY_HEIGHT, left, top))
        self.frame = Frame(self.root)

        self.root.wm_iconbitmap(bitmap = "@media/icon.xbm")

        # Too much hassle on Windows
        if 'nt' != os.name:
            self.root.tk.call('wm', 'iconphoto', self.root._w, PhotoImage('icon.xbm'))

        self.root.protocol('WM_DELETE_WINDOW', self.quit)

        # Create panes for two rows of widgets
        self.pane1 = self._add_pane()
        self.pane2 = self._add_pane()

        # Add a buttons
        self.button_connect = self._add_button('Connect', self.pane1, self._connect_callback)
        self.button_imu  = self._add_button('IMU',  self.pane2, self._imu_callback)
        self.button_motors = self._add_button('Motors', self.pane2, self._motors_button_callback)
        self.button_receiver = self._add_button('Receiver', self.pane2, self._receiver_button_callback)
        #self.button_messages = self._add_button('Messages', self.pane2, self._messages_button_callback)
        #self.button_maps = self._add_button('Maps', self.pane2, self._maps_button_callback, disabled=False)

        # Prepare for adding ports as they are detected by our timer task
        self.portsvar = StringVar(self.root)
        self.portsmenu = None
        self.connected = False
        self.ports = []

        # Finalize Tk stuff
        self.frame.pack()
        self.canvas = Canvas(self.root, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, background='black')
        self.canvas.pack()


        # Set up a text label for reporting errors
        errmsg = 'No response from board.  Possible reasons:\n\n' + \
                 '    * You connected to the wrong port.\n\n' + \
                 '    * Firmware uses serial receiver\n' + \
                 '      (DSMX, SBUS), but receiver is\n' + \
                 '      not connected.'
        self.error_label = Label(self.canvas, text=errmsg, bg='black', fg='red', font=(None,24), justify=LEFT)
        self.hide(self.error_label)

        # Add widgets for motor-testing dialog; hide them immediately
        self.motors = Motors(self)
        self.motors.stop()

        # Create receiver dialog
        self.receiver = Receiver(self)

        # Create messages dialog
        #self.messages = Messages(self)

        # Create IMU dialog
        self.imu = IMU(self)
        self._schedule_connection_task()

        # Create a maps dialog
        #self.maps = Maps(self, yoffset=-30)

        # Create a splash image
        self.splashimage = PhotoImage(file='media/splash.gif')
        self._show_splash()

        # Create a message parser 
        self.parser = MSP_Parser()

        # Set up parser's request strings
        self.attitude_request = serialize_ATTITUDE_RADIANS_Request()
        self.rc_request = serialize_RC_NORMAL_Request()

        # No messages yet
        self.roll_pitch_yaw = 0,0,0
        self.rxchannels = 0,0,0,0,0,0,0,0

        # A hack to support display in IMU dialog
        self.active_axis = 0

    def quit(self):
        self.motors.stop()
        self.root.destroy()

    def hide(self, widget):

        widget.place(x=-9999)

    def getChannels(self):

        return self.rxchannels

    def getRollPitchYaw(self):

        # configure button to show connected
        self._enable_buttons()
        self.button_connect['text'] = 'Disconnect'
        self._enable_button(self.button_connect)

        return self.roll_pitch_yaw

    def checkArmed(self):

        if self.armed:

            self._show_armed(self.root)
            self._show_armed(self.pane1)
            self._show_armed(self.pane2)

            self._disable_button(self.button_motors)

        else:

            self._show_disarmed(self.root)
            self._show_disarmed(self.pane1)
            self._show_disarmed(self.pane2)

    def scheduleTask(self, delay_msec, task):

        self.root.after(delay_msec, task)

    def _add_pane(self):

        pane = PanedWindow(self.frame, bg=BACKGROUND_COLOR)
        pane.pack(fill=BOTH, expand=1)
        return pane

    def _add_button(self, label, parent, callback, disabled=True):

        button = Button(parent, text=label, command=callback)
        button.pack(side=LEFT)
        button.config(state = 'disabled' if disabled else 'normal')
        return button

    # Callback for IMU button
    def _imu_callback(self):

        self._clear()

        self.motors.stop()
        self.receiver.stop()
        #self.messages.stop()
        #self.maps.stop()

        self.parser.set_ATTITUDE_RADIANS_Handler(self._handle_attitude)
        self._send_attitude_request()
        self.imu.start()

    def _start(self):

        self.parser.set_ATTITUDE_RADIANS_Handler(self._handle_attitude)
        self._send_attitude_request()
        self.imu.start()

        self.parser.set_RC_NORMAL_Handler(self._handle_rc)

        self.gotimu = False
        self.hide(self.error_label)
        self.scheduleTask(BOARD_REPLY_DELAY_MSEC, self._checkimu)

    def _checkimu(self):

        if not self.gotimu:

            self.imu.stop()
            self.error_label.place(x=50, y=50)
            self._disable_button(self.button_imu)
            self._disable_button(self.button_motors)
            self._disable_button(self.button_receiver)

    # Sends Attitude request to FC
    def _send_attitude_request(self):

        self.comms.send_request(self.attitude_request)

    # Sends RC request to FC
    def _send_rc_request(self):

        self.comms.send_request(self.rc_request)

    # Callback for Motors button
    def _motors_button_callback(self):

        self._clear()

        self.imu.stop()
        self.receiver.stop()
        #self.messages.stop()
        #self.maps.stop()
        self.motors.start()

    def _clear(self):

        self.canvas.delete(ALL)

    # Callback for Receiver button
    def _receiver_button_callback(self):

        self._clear()

        self.imu.stop()
        self.motors.stop()
        #self.messages.stop()
        #self.maps.stop()

        self._send_rc_request()
        self.receiver.start()

    # Callback for Messages button
    def _messages_button_callback(self):

        self._clear()

        self.imu.stop()
        self.motors.stop()
        #self.maps.stop()
        self.receiver.stop()

        self.messages.start()

    def _getting_messages(self):

        return self.button_connect['text'] == 'Disconnect'

    # Callback for Maps button
    def _maps_button_callback(self):

        self._clear()

        if self._getting_messages():

            self.receiver.stop()
            self.messages.stop()
            self.imu.stop()
            self.motors.stop()

        #self.maps.start()

    # Callback for Connect / Disconnect button
    def _connect_callback(self):

        if self.connected:

            self.imu.stop()
            #self.maps.stop()
            self.motors.stop()
            #self.messages.stop()
            self.receiver.stop()

            if not self.comms is None:

                self.comms.stop()

            self._clear()

            self._disable_buttons()

            self.button_connect['text'] = 'Connect'

            self.hide(self.error_label)

            self._show_splash()

        else:

            #self.maps.stop()

            self.comms = Comms(self)
            self.comms.start()

            self.button_connect['text'] = 'Connecting ...'
            self._disable_button(self.button_connect)

            self._hide_splash()

            self.scheduleTask(CONNECTION_DELAY_MSEC, self._start)

        self.connected = not self.connected

    # Gets available ports
    def _getports(self):

        allports = comports()

        ports = []

        for port in allports:
            
            portname = port[0]

            if 'ttyACM' in portname or 'ttyUSB' in portname or 'COM' in portname:
                ports.append(portname)

        return ports

    # Checks for changes in port status (hot-plugging USB cables)
    def _connection_task(self):

        ports = self._getports()

        if ports != self.ports:

            if self.portsmenu is None:

                self.portsmenu = OptionMenu(self.pane1, self.portsvar, *ports)

            else:

                for port in ports:

                    self.portsmenu['menu'].add_command(label=port)

            self.portsmenu.pack(side=LEFT)

            if ports == []:

                self._disable_button(self.button_connect)
                self._disable_buttons()

            else:
                self.portsvar.set(ports[0]) # default value
                self._enable_button(self.button_connect)

            self.ports = ports

        self._schedule_connection_task()

    # Mutually recursive with above
    def _schedule_connection_task(self):

        self.root.after(USB_UPDATE_MSEC, self._connection_task)

    def _disable_buttons(self):

        self._disable_button(self.button_imu)
        self._disable_button(self.button_motors)
        self._disable_button(self.button_receiver)
        #self._disable_button(self.button_messages)

    def _enable_buttons(self):

        self._enable_button(self.button_imu)
        self._enable_button(self.button_motors)
        self._enable_button(self.button_receiver)
        #self._enable_button(self.button_messages)

    def _enable_button(self, button):

        button['state'] = 'normal'

    def _disable_button(self, button):

        button['state'] = 'disabled'

    def sendMotorMessage(self, index, percent):

        values = [0]*4
        values[index-1] = percent / 100.
        self.comms.send_message(serialize_SET_MOTOR_NORMAL, values)

    def _show_splash(self):

        self.splash = self.canvas.create_image((400,250), image=self.splashimage)

    def _hide_splash(self):

        self.canvas.delete(self.splash)

    def _show_armed(self, widget):

        widget.configure(bg='red')

    def _show_disarmed(self, widget):

        widget.configure(bg=BACKGROUND_COLOR)

        if self._getting_messages():

            self._enable_button(self.button_motors)

    def _handle_calibrate_response(self):

        self.imu.showCalibrated()

    def _handle_params_response(self, pitchroll_kp_percent, yaw_kp_percent):

        # Only handle parms from firmware on a fresh connection
        if self.newconnect:

            self.imu.setParams(pitchroll_kp_percent, yaw_kp_percent)

        self.newconnect = False

    def _handle_attitude(self, x, y, z):

        self.roll_pitch_yaw = x, -y, z  

        self.gotimu = True

        #self.messages.setCurrentMessage('Roll/Pitch/Yaw: %+3.3f %+3.3f %+3.3f' % self.roll_pitch_yaw)

        # As soon as we handle the callback from one request, send another request, if IMU dialog is running
        if self.imu.running:
            self._send_attitude_request()

    def _handle_rc(self, c1, c2, c3, c4, c5, c6, c7, c8):

        # Display throttle as [0,1], other channels as [-1,+1]
        self.rxchannels = c1/2.+.5, c2, c3, c4, c5

        # As soon as we handle the callback from one request, send another request, if receiver dialog is running
        if self.receiver.running:
            self._send_rc_request()

        #self.messages.setCurrentMessage('Receiver: %04d %04d %04d %04d %04d' % (c1, c2, c3, c4, c5))

    def _handle_arm_status(self, armed):

        self.armed = armed

        #self.messages.setCurrentMessage('ArmStatus: %s' % ('ARMED' if armed else 'Disarmed'))

    def _handle_battery_status(self, volts, amps):

        return
        #self.messages.setCurrentMessage('BatteryStatus: %3.3f volts, %3.3f amps' % (volts, amps))

# Comms class for communiating with flight controller ====================================================

class Comms:

    def __init__(self, gcs):

        self.gcs = gcs

        portname = gcs.portsvar.get()

        baud = BAUD

        self.port = Serial(portname, baud)

        self.thread = Thread(target=self.run)
        self.thread.setDaemon(True)

        self.running = False

    def send_message(self, serializer, contents):

        for c in serializer(*contents):

            self.port.write(c)

    def send_request(self, request):

        self.port.write(request)

    def run(self):

        while self.running:
            try:
                byte = self.port.read(1)
                self.gcs.parser.parse(byte)
            except:
                None

    def start(self):

        self.running = True

        self.thread.start()

        self.gcs.newconnect = True

    def stop(self):

        self.running = False

        self.port.close()

# Main ==============================================================================================================

if __name__ == "__main__":

    gcs = GCS()

    mainloop()
