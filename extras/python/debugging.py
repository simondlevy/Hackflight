'''
Debugging support

Copyright (c) 2021 Simon D. Levy

MIT License
'''

from sys import stdout


def debug(message):
    print(message)
    stdout.flush()
