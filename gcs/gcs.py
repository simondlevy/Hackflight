#!/usr/bin/python3
'''
Hackflight Ground Control Station main program

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
'''

from serial import Serial

from msp import Parser

class MyParser(Parser):

    def handle_STATE(self, foo):

        print('Got: ', foo)


def main():

    port = Serial('/dev/ttyUSB0', 115200)

    parser = MyParser()

    while True:

        try:

            c = port.read(1)

            print(ord(c))

            parser.parse(c)

        except KeyboardInterrupt:

            break

main()
