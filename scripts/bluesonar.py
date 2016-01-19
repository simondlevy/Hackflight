#!/usr/bin/env python3

BT_BAUD = 115200
BT_ADDR = "00:06:66:73:e3:a6"
BT_PORT = 1

SONARMIN = 0
SONARMAX = 200
BAROMAX = 10
 
from msppg import MSP_Parser as Parser
from realtime_plot import RealtimePlotter
from time import localtime, strftime, sleep
import bluetooth
import threading

class AGLPlotter(RealtimePlotter):

    def __init__(self):

        agllim = SONARMIN, SONARMAX

        RealtimePlotter.__init__(self, [(SONARMIN,+SONARMAX), (-BAROMAX,+BAROMAX)], 
                window_name='Altitude Sensor Fusion',
                yticks = [range(SONARMIN,+SONARMAX,50), range(-BAROMAX,+BAROMAX,1)],
                styles = [('r','b'), 'g'], 
                legends = [('Sonar', 'Fused'), None],
                ylabels=['AGL (cm)', 'Baro (mb)'])

        self.xcurr = 0
        self.fused = 0
        self.baropress = 0
        self.sonar = 0
 
        self.parser = Parser()
        self.parser.set_MB1242_Handler(self.handler)
        self.request = self.parser.serialize_MB1242_Request()

        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM )
        self.sock.connect((BT_ADDR, BT_PORT))
        self.sock.send(self.request)

        print('connected to %s' % BT_ADDR)

        self.logfile = open('logs/' + strftime("%d-%b-%Y-%H-%M-%S.csv", localtime()), 'w')

    def handler(self, baropress, sonar):
        self.fused = 0
        self.baropress = baropress / 100.
        self.sonar = sonar
        self.logfile.write('%d,%d\n' % (baropress, sonar))
        self.logfile.flush()
        self.sock.send(self.request)

    def getValues(self):

        return self.sonar, self.fused, self.baropress

    def update(self):

        while True:

            self.parser.parse(self.sock.recv(1))
            plotter.xcurr += 1
            sleep(.001)

if __name__ == '__main__':

    plotter = AGLPlotter()

    thread = threading.Thread(target=plotter.update)
    thread.daemon = True

    thread.start()
    plotter.start()
