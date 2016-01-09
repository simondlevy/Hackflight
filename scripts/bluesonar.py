#!/usr/bin/env python3

BT_BAUD = 115200
BT_ADDR = "00:06:66:73:e3:a6"
BT_PORT = 1

MAXAGLCM = 300
MAXPID   = 250
 
from msppg import MSP_Parser as Parser
from realtime_plot import RealtimePlotter
from time import localtime, strftime, sleep
import bluetooth
import threading

class AGLPlotter(RealtimePlotter):

    def __init__(self):

        ylim = (0, MAXAGLCM)
        ytic = range(ylim[0], ylim[1], 20)

        RealtimePlotter.__init__(self, [ylim, (-MAXPID,+MAXPID)], 
                window_name='MB1242 Sonar',
                yticks = [ytic, range(-MAXPID, +MAXPID, 25)],
                styles = [('b', 'r'), 'k'], 
                ylabels=['AGL (cm)', 'PID'])

        self.xcurr = 0
        self.aglcm = 0
        self.holdcm = 0
        self.pid = 0

        self.parser = Parser()
        self.parser.set_MB1242_Handler(self.handler)
        self.request = self.parser.serialize_MB1242_Request()

        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM )
        self.sock.connect((BT_ADDR, BT_PORT))
        self.sock.send(self.request)

        print('connected to %s' % BT_ADDR)

        self.logfile = open('logs/' + strftime("%d-%b-%Y-%H-%M-%S.csv", localtime()), 'w')

    def handler(self, aglcm, holdcm, pid):
        self.aglcm = aglcm
        self.holdcm = holdcm
        self.pid = pid
        self.logfile.write('%d,%d,%d\n' % (aglcm, holdcm, pid))
        self.logfile.flush()
        self.sock.send(self.request)

    def getValues(self):

        return (self.aglcm,self.holdcm,self.pid)

    def update(self):

        while True:

            self.parser.parse(self.sock.recv(1))
            plotter.xcurr += 1
            #sleep(.002)
            sleep(.001)


if __name__ == '__main__':

    plotter = AGLPlotter()

    thread = threading.Thread(target=plotter.update)
    thread.daemon = True

    thread.start()
    plotter.start()
