'''
Serial communications support for  Hackflight GCS

Copyright (C) Simon D. Levy 2021

MIT License
'''

from serial import Serial
from threading import Thread

BAUD = 115200


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
                self.gcs.parse(byte)
            except Exception:
                None

    def start(self):

        self.running = True

        self.thread.start()

        self.gcs.newconnect = True

    def stop(self):

        self.running = False

        self.port.close()
