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

import time
from threading import Thread

from gamepad import Gamepad
from telemetry import Telemetry


def telemetry_threadfun(telemetry):

    while True:

        telemetry.step()

        time.sleep(0)  # yield


def main():

    # Telemetry parser does all the initialization
    telemetry = Telemetry()

    thread = Thread(target=telemetry_threadfun, args=(telemetry, ))
    thread.daemon = True
    thread.start()

    gamepad = Gamepad(telemetry.port)

    while gamepad.connected:

        gamepad.step()

main()
