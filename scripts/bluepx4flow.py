#!/usr/bin/env python3

BT_BAUD = 115200
BT_ADDR = "00:06:66:73:e3:a6"
BT_PORT = 1

MAXFLOW  = 30
MAXPHASE = 300

from realtime_plot import RealtimePlotter
from msppg import MSP_Parser as Parser
from time import localtime, strftime, sleep
import bluetooth
import threading

class PX4FlowPlotter(RealtimePlotter):

    def __init__(self):

        ticks = range(-MAXFLOW, +MAXFLOW, 5)
        lims  = -MAXFLOW,+MAXFLOW
        phaselims = -MAXPHASE, MAXPHASE

        RealtimePlotter.__init__(self, [lims, lims],
                phaselims = [phaselims, phaselims],
                window_name='PX4Flow',
                styles = ['b', 'r'], 
                yticks = [ticks, ticks],
                ylabels=['X', 'Y'])

        self.xcurr = 0

        self.pixel_flow_x_sum = 0
        self.pixel_flow_y_sum = 0

        self.xacc = 0
        self.yacc = 0

        self.parser = Parser()
        self.parser.set_PX4FLOW_Handler(self.handler)
        self.request = self.parser.serialize_PX4FLOW_Request()

        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM )
        self.sock.connect((BT_ADDR, BT_PORT))
        self.sock.send(self.request)

        print('connected to %s' % BT_ADDR)

        self.logfile = open('logs/px4flow-' + strftime("%d-%b-%Y-%H-%M-%S", localtime()), 'w')

    def handler(self, pixel_flow_x_sum, pixel_flow_y_sum):
        self.pixel_flow_x_sum = pixel_flow_x_sum
        self.pixel_flow_y_sum = pixel_flow_y_sum
        self.xacc += pixel_flow_x_sum
        self.yacc += pixel_flow_y_sum
        self.logfile.write('%d %d %d %d\n' % (pixel_flow_x_sum, pixel_flow_y_sum, self.xacc, self.yacc))
        self.logfile.flush()
        self.sock.send(self.request)

    def getValues(self):

        return self.xacc, self.yacc, self.pixel_flow_x_sum, self.pixel_flow_y_sum

    def update(self):

        while True:

            self.parser.parse(self.sock.recv(1))
            plotter.xcurr += 1
            sleep(.001)
            
if __name__ == '__main__':

    plotter = PX4FlowPlotter()

    thread = threading.Thread(target=plotter.update)
    thread.daemon = True

    thread.start()
    plotter.start()
