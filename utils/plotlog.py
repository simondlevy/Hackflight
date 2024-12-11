#!/usr/bin/python3
'''
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

import argparse


def main():

    fmtr = argparse.ArgumentDefaultsHelpFormatter

    arg_parser = argparse.ArgumentParser(formatter_class=fmtr)

    arg_parser.add_argument('-f', '--file', default='/media/levys/boot/log.dat')

    args = arg_parser.parse_args()

    with open(args.file, 'rb') as file:

        while True:

            buf = file.read(48)  # 12 floats

            if len(buf) < 48:

                break

            print('okay')

main()
