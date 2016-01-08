#!/usr/bin/env python3

BT_BAUD = 115200
BT_ADDR = "00:06:66:73:e3:a6"
BT_PORT = 1

MAXAGLCM = 300

from msppg import MSP_Parser as Parser
from realtime_plot import RealtimePlotter
from time import localtime, strftime, sleep
import bluetooth
import threading

class AGLPlotter(RealtimePlotter):

    def __init__(self):

        ylim = (0, MAXAGLCM)
        ytic = range(ylim[0], ylim[1], 20)
        RealtimePlotter.__init__(self, [ylim, ylim], 
                window_name='MB1242 Sonar',
                yticks = [ytic, ytic],
                styles = ['b', 'r'], 
                ylabels=['AGL (cm)', 'HOLD (cm)'])

        self.xcurr = 0
        self.aglcm = 0
        self.holdcm = 0

        self.parser = Parser()
        self.parser.set_MB1242_Handler(self.handler)
        self.request = self.parser.serialize_MB1242_Request()

        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM )
        self.sock.connect((BT_ADDR, BT_PORT))
        self.sock.send(self.request)

        print('connected to %s' % BT_ADDR)

        self.logfile = open('logs/' + strftime("%d-%b-%Y-%H-%M-%S", localtime()), 'w')

    def handler(self, aglcm, holdcm):
        print(aglcm, holdcm)
        self.aglcm = aglcm
        self.holdcm = holdcm
        self.logfile.write('%d %d\n' % (aglcm, holdcm))
        self.sock.send(self.request)

    def getValues(self):

        return (self.aglcm,self.holdcm)

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
