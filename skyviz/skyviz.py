#!/usr/bin/python3

from argparse import ArgumentParser
from serial import Serial
from struct import unpack

from mspparser import MspParser


class SkyParser(MspParser):

    def dispatchMessage(self):

        # Attitude
        if self.message_id == 213:
            print('attitude:', *unpack('=hhh', self.message_buffer))

        '''
        # VL53L5 ranging camera
        if self.message_id == 221:
            print('ranger:  ', *unpack('=hhhhhhhhhhhhhhhh', self.message_buffer))

        # PAA3905 mocap
        if self.message_id == 222:
            print('mocap:   ', *unpack('=hh', self.message_buffer))
        '''


def main():

    cmdparser = ArgumentParser()

    cmdparser.add_argument('-p', '--port',
                           default='/dev/ttyUSB1',
                           help='COM port')

    args = cmdparser.parse_args()

    port = Serial(args.port, 115200)

    skyparser = SkyParser()

    while True:

        try:

            skyparser.parse(port.read(1))

        except KeyboardInterrupt:

            break


main()
