"""
Simple debugging support for Hackflight

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy

MIT License
"""


from sys import stdout


def debug(msg):
    print(msg)
    stdout.flush()
