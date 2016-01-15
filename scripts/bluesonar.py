#!/usr/bin/env python3

BT_BAUD = 115200
BT_ADDR = "00:06:66:73:e3:a6"
BT_PORT = 1

MAXAGLCM = 200
 
from msppg import MSP_Parser as Parser
from realtime_plot import RealtimePlotter
from time import localtime, strftime, sleep
import bluetooth
import threading

class AGLPlotter(RealtimePlotter):

    def __init__(self):

        ylim = (-MAXAGLCM, MAXAGLCM)
        ytic = range(ylim[0], ylim[1], 20)

        RealtimePlotter.__init__(self, [ylim], 
                window_name='MB1242 Sonar',
                yticks = [ytic],
                styles = [('r', 'g', 'b')], 
                legends = [('accel', 'baro', 'sonar')],
                ylabels=['AGL (cm)'])

        self.xcurr = 0
        self.accel = 0
        self.baro = 0
        self.sonar = 0
 
        self.parser = Parser()
        self.parser.set_MB1242_Handler(self.handler)
        self.request = self.parser.serialize_MB1242_Request()

        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM )
        self.sock.connect((BT_ADDR, BT_PORT))
        self.sock.send(self.request)

        print('connected to %s' % BT_ADDR)

        self.logfile = open('logs/' + strftime("%d-%b-%Y-%H-%M-%S.csv", localtime()), 'w')

    def handler(self, accel, baro, sonar):
        self.accel = accel
        self.baro = baro
        self.sonar = sonar
        self.logfile.write('%d,%d,%d\n' % (accel, baro, sonar))
        self.logfile.flush()
        self.sock.send(self.request)

    def getValues(self):

        return self.accel, self.baro, self.sonar

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
