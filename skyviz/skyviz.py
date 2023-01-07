#!/usr/bin/python3

from argparse import ArgumentParser
from serial import Serial

from mspparser import MspParser


class SkyParser(MspParser):

    def dispatchMessage(self):

        print(self.message_id)


def main():

    cmdparser = ArgumentParser()
    cmdparser.add_argument('-p', '--port',
                           default='/dev/ttyUSB0',
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
