#!/usr/bin/env python3

# for Bluetooth
BT_ADDR = "00:06:66:73:e3:a6"
BT_PORT = 1

# for plotting
BARO_RANGE    = 20
SONAR_RANGE   = 200

from altitude_fuser import ASL_EKF, ASL_Plotter
from msppg import MSP_Parser as Parser
import threading
import bluetooth

class Bluetooth_ASL_EKF(ASL_EKF):

    def __init__(self, plotter):

        ASL_EKF.__init__(self)

        self.plotter = plotter

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

        while self.plotter.running:

            self.parser.parse(self.sock.recv(1))

    def handler(self, baro, sonar):

        print(baro, sonar)

        self.sock.send(self.request)

    def getBaroBaseline(self):

        return 1974822

class Bluetooth_ASLPlotter(ASL_Plotter):

    def __init__(self):

        self.running = True

        ASL_Plotter.__init__(self, Bluetooth_ASL_EKF(self))

    def handleClose(self, event):

        ASL_Plotter.handleClose(self, event)

        self.running = False

    def getSensors(self):

        return 0, 0

if __name__ == '__main__':

    plotter = Bluetooth_ASLPlotter()

    thread = threading.Thread(target=plotter.update)
    thread.daemon = True

    thread.start()
    plotter.start()
