#!/usr/bin/env python3

BT_BAUD = 115200
BT_ADDR = "00:06:66:73:e3:a6"
BT_PORT = 1

MAX_AGL      = 200
MIN_PRESSURE = 97700
MAX_PRESSURE = 97800
 
from msppg import MSP_Parser as Parser
from realtime_plot import RealtimePlotter

from time import localtime, strftime, sleep
import threading
import socket

class AGLPlotter(RealtimePlotter):

    def __init__(self):

        RealtimePlotter.__init__(self, [(MIN_PRESSURE,MAX_PRESSURE), (0,MAX_AGL)],
                window_name='Altitude Estimation',
                styles = ['r', 'b'], 
                ylabels=['Pressure (kPa)', 'AGL (cm)'])

        self.xcurr = 0
        self.baro = 0
        self.sonar = 0
 
        self.parser = Parser()
        self.parser.set_BARO_SONAR_RAW_Handler(self.handler)
        self.request = self.parser.serialize_BARO_SONAR_RAW_Request()

        self.sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.sock.connect((BT_ADDR, BT_PORT))
        self.sock.send(self.request)

        print('connected to %s' % BT_ADDR)

        #self.logfile = open('logs/' + strftime("%d-%b-%Y-%H-%M-%S.csv", localtime()), 'w')

    def handler(self, baro, sonar):
        self.baro = baro
        self.sonar = sonar
        #self.logfile.write('%d,%d\n' % (baro, sonar))
        #self.logfile.flush()
        self.sock.send(self.request)

    def getValues(self):

        return self.baro, self.sonar

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
