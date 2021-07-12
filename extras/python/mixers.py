'''
   Motor mixer classes

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy

MIT License
'''

from demands import DEMANDS_THROTTLE, DEMANDS_ROLL, DEMANDS_PITCH, DEMANDS_YAW
import numpy as np


def _mixer(motordirs, demands):

    m = len(motordirs)

    motorvals = np.zeros(m)

    for i in range(m):

        motorvals[i] = (demands[DEMANDS_THROTTLE] * motordirs[i]['throttle'] +
                        demands[DEMANDS_ROLL] * motordirs[i]['roll'] +
                        demands[DEMANDS_PITCH] * motordirs[i]['pitch'] +
                        demands[DEMANDS_YAW] * motordirs[i]['yaw'])

    # Keep motor values in appropriate interval
    motorvals[motorvals < 0] = 0
    motorvals[motorvals > 1] = 1

    return motorvals


def mixer_quadxap(demands):
    '''
    Mixer for X-configuration quadcopters following the ArduPilot numbering
    convention:

    3cw   1ccw
       | /
        ^
       / |
    2ccw  4cw
    '''

    MOTORDIRS = (
                 {'throttle': +1, 'roll': -1, 'pitch': -1, 'yaw': +1},
                 {'throttle': +1, 'roll': +1, 'pitch': +1, 'yaw': +1},
                 {'throttle': +1, 'roll': +1, 'pitch': -1, 'yaw': -1},
                 {'throttle': +1, 'roll': -1, 'pitch': +1, 'yaw': -1}
                )

    return _mixer(MOTORDIRS, demands)


def mixer_coaxial(demands):
    '''
    Hypothetical mixer for coaxial vehicle
    '''
    MOTORDIRS = (
                 {'throttle': +1, 'roll': 0, 'pitch': 0, 'yaw': +1},
                 {'throttle': +1, 'roll': 0, 'pitch': 0, 'yaw': -1},
                 {'throttle': 0, 'roll': +1, 'pitch': 0, 'yaw': 0},
                 {'throttle': 0, 'roll': 0, 'pitch': +1, 'yaw': 0}
                )

    return _mixer(MOTORDIRS, demands)
