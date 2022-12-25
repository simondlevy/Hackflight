#!/usr/bin/python3

import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-p', '--port', default='/dev/ttyUSB0', help='COM port')
args = parser.parse_args()

print(args.port)
