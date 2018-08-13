#!/usr/bin/env python

'''
comms.py : serial communications support for  Hackflight GCS

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

from serial import Serial
from threading import Thread

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

        self.port.write(serializer(*contents))

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
