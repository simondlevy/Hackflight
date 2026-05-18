#!/usr/bin/python3
'''
Copyright (C) 2026 Simon D. Levy

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, in version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
'''

import argparse
from argparse import ArgumentDefaultsHelpFormatter
import serial
from time import sleep
from threading import Thread

from radiomaster import RadioMaster
from telemetry import TelemetryParser


def telemetry_threadfun(port, outfile):

    telemetryParser = TelemetryParser(outfile)

    if outfile is not None:
        print('Connected')

    while True:
        try:
            telemetryParser.parse(port.read(1))
        except serial.SerialException:
            print('Unable to read telemtry from port')

        sleep(0)  # yield


def main():

    argparser = argparse.ArgumentParser(
                formatter_class=ArgumentDefaultsHelpFormatter)

    argparser.add_argument('-o', '--outfile', help='CSV file for logging')

    argparser.add_argument('-p', '--port', default='/dev/ttyUSB0',
                           help='Serial port for dongle')

    args = argparser.parse_args()

    try:
        port = serial.Serial(args.port, 115200)

    except serial.SerialException:
        print('Unable to open port ' + args.port)
        exit(1)

    outfile = None

    if args.outfile is not None:
        try:
            outfile = open(args.outfile, 'w')
        except Exception as e:
            print('Unable to open log file %s: %s' % (args.outfile, str(e)))
            exit(1)


    print('Waiting for server ... ', end='')

    telemetry_thread = Thread(target=telemetry_threadfun, args=(port, outfile))
    telemetry_thread.daemon = True
    telemetry_thread.start()

    rm = RadioMaster(port)

    while rm.connected:

        try:
            rm.step()

        except KeyboardInterrupt:
            break


main()
