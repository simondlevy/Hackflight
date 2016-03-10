#!/usr/bin/env python3

BT_ADDR = "00:06:66:73:e3:a6"
BT_PORT = 1

MAXAGLCM = 200

from msppg import MSP_Parser as Parser
from realtime_plot import RealtimePlotter
from time import localtime, strftime, sleep
import bluetooth
import threading
import numpy as np

class AGLPlotter(RealtimePlotter):

    def __init__(self):

        RealtimePlotter.__init__(self, [(0,MAXAGLCM)], 
                window_name='LidarLite AGL',
                yticks = [range(0,MAXAGLCM,20)],
                styles = ['r'], 
                ylabels=['AGL (cm)'])

        self.xcurr = 0
        self.aglcm = 0

        self.parser = Parser()
        self.parser.set_LIDARLITE_Handler(self.handler)
        self.request = self.parser.serialize_LIDARLITE_Request()

        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM )
        self.sock.connect((BT_ADDR, BT_PORT))
        self.sock.send(self.request)

        print('connected to %s' % BT_ADDR)

        self.logfile = open('logs/' + strftime("%d-%b-%Y-%H-%M-%S", localtime()), 'w')

    def handler(self, aglcm):
        print(aglcm)
        self.aglcm = aglcm
        self.logfile.write('%d\n' % aglcm)
        self.sock.send(self.request)

    def getValues(self):

        return (self.aglcm,)

    def update(self):

        while True:

            self.parser.parse(self.sock.recv(1))
            plotter.xcurr += 1
            sleep(.002)


if __name__ == '__main__':

    plotter = AGLPlotter()

    thread = threading.Thread(target=plotter.update)
    thread.daemon = True

    thread.start()
    plotter.start()
