#!/usr/bin/env python3

# for Bluetooth
BT_ADDR = "00:06:66:73:e3:a6"
BT_PORT = 1

from altitude_fuser import ASL_Plotter
from msppg import MSP_Parser as Parser
import threading
import bluetooth

class Bluetooth_ASLPlotter(ASL_Plotter):

    def __init__(self):

        ASL_Plotter.__init__(self)

        self.running = True

        self.actual_baro = 0
        self.actual_sonar = 0
        
        self.parser = Parser()
        self.parser.set_MB1242_Handler(self.handler)
        self.request = self.parser.serialize_MB1242_Request()

        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM )
        self.sock.connect((BT_ADDR, BT_PORT))

        print('connected to %s' % BT_ADDR)

        thread = threading.Thread(target = self.loop)
        thread.daemon = True
        thread.start()

        self.sock.send(self.request)

    def loop(self):

        while self.running:

            self.parser.parse(self.sock.recv(1))

    def handler(self, baro, sonar):

        self.actual_baro = baro
        self.actual_sonar = sonar

        self.sock.send(self.request)

    def handleClose(self, event):

        ASL_Plotter.handleClose(self, event)

        self.running = False

    def getSensors(self):

        print(self.actual_baro, self.actual_sonar)

        return 0, 0

if __name__ == '__main__':

    plotter = Bluetooth_ASLPlotter()

    thread = threading.Thread(target=plotter.update)
    thread.daemon = True

    thread.start()
    plotter.start()
