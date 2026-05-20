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


def radio_threadfun(rm):
    while rm.connected:
        rm.step()
        sleep(0)  # yield


def launch_thread(threadfun, arg):
    thread = Thread(target=threadfun, args=(arg, ))
    thread.daemon = True
    thread.start()


def main():

    telemetryParser = TelemetryParser()

    port = telemetryParser.port

    rm = RadioMaster(port)

    # If we're plotting telemetry, we need to run the setpoint control on
    # its own thread
    if telemetryParser.plotter is not None:

        launch_thread(radio_threadfun, rm)

        telemetryParser.plotter.start()

    # No telemetry; run setpoint control on main thread
    else:

        launch_thread(telemetry_threadfun, telemetryParser)

        while rm.connected:

            try:
                rm.step()

            except KeyboardInterrupt:
                break


main()
