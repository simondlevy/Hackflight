'''
   Motor mixer classes

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy

MIT License
'''

from demands import DEMANDS_THROTTLE, DEMANDS_ROLL, DEMANDS_PITCH, DEMANDS_YAW
import numpy as np

'''
Mixer for X-configuration quadcopters following the ArduPilot numbering
convention:

3cw   1ccw
   | /
    ^
   / |
2ccw  4cw
'''
QUADXAP_MOTORDIRS = (
                     {'throttle': +1, 'roll': -1, 'pitch': -1, 'yaw': +1},
                     {'throttle': +1, 'roll': +1, 'pitch': +1, 'yaw': +1},
                     {'throttle': +1, 'roll': +1, 'pitch': -1, 'yaw': -1},
                     {'throttle': +1, 'roll': -1, 'pitch': +1, 'yaw': -1}
                    )


'''
Hypothetical mixer for coaxial vehicle
'''
COAXIAL_MOTORDIRS = (
                     {'throttle': +1, 'roll': 0, 'pitch': 0, 'yaw': +1},
                     {'throttle': +1, 'roll': 0, 'pitch': 0, 'yaw': -1},
                     {'throttle': 0, 'roll': +1, 'pitch': 0, 'yaw': 0},
                     {'throttle': 0, 'roll': 0, 'pitch': +1, 'yaw': 0}
                    )


def mixerfun(motordirs, demands):

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

    return motorvals.copy()
