'''
Debugging support for Hackflight GCS

Copyright (C) Simon D. Levy 2021

MIT License
'''

from sys import stdout


def debug(msg):
    print(msg)
    stdout.flush()
