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
Hackflight. If not, see <https://www.gnu.org/licenses/>.  '''

from serial import Serial
from threading import Thread

BAUD = 115200


class Comms:

    def __init__(self, viz):

        self.viz = viz

        portname = viz.portsvar.get()

        baud = BAUD

        self.port = Serial(portname, baud)

        self.thread = Thread(target=self.run)
        self.thread.setDaemon(True)

        self.running = False

    def send_message(self, serializer, contents):

        msg = serializer(*contents)

        if self.port.isOpen():
            self.port.write(msg)

    def send_request(self, request):

        self.port.write(request)

    def run(self):

        while self.running:
            try:
                byte = self.port.read(1)
                self.viz.parse(byte)
            except Exception:
                None

    def start(self):

        self.running = True

        self.thread.start()

        self.viz.newconnect = True

    def stop(self):

        self.running = False

        self.port.close()
