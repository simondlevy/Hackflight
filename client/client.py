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

from time import sleep
from threading import Thread

from radiomaster import RadioMaster
from telemetry import TelemetryParser


def telemetry_threadfun(telemetryParser):

    while True:

        telemetryParser.step()

        sleep(0)  # yield


def radio_threadfun(port):

    rm = RadioMaster(port)

    count = 0

    while rm.connected:

        rm.step()

        print(count)
        count += 1

        sleep(0)  # yield

def main():

    telemetryParser = TelemetryParser()

    if telemetryParser.plotter is not None:

        radio_thread = Thread(target=radio_threadfun,
                                  args=(telemetryParser.port, ))
        radio_thread.daemon = True
        radio_thread.start()

        telemetryParser.plotter.start()

    else:

        telemetry_thread = Thread(target=telemetry_threadfun,
                                  args=(telemetryParser, ))
        telemetry_thread.daemon = True
        telemetry_thread.start()

        rm = RadioMaster(telemetryParser.port)

        while rm.connected:

            try:
                rm.step()

            except KeyboardInterrupt:
                break


main()
