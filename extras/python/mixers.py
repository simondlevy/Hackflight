'''
   Motor mixer classes

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy

MIT License
'''

from demands import DEMANDS_THROTTLE, DEMANDS_ROLL, DEMANDS_PITCH, DEMANDS_YAW
import numpy as np


class Mixer(object):

    def __init__(self, motordirs):

        self.motordirs = motordirs

    def getMotors(self, demands):

        # Map throttle demand from [-1,1] to [0,1]
        demands[DEMANDS_THROTTLE] = (demands[DEMANDS_THROTTLE]+1)/2

        m = len(self.motordirs)

        motorvals = np.zeros(m)

        for i in range(m):

            motorvals[i] = (demands[DEMANDS_THROTTLE] *
                            self.motordirs[i]['throttle'] +

                            demands[DEMANDS_ROLL] *
                            self.motordirs[i]['roll'] +

                            demands[DEMANDS_PITCH] *
                            self.motordirs[i]['pitch'] +

                            demands[DEMANDS_YAW] *
                            self.motordirs[i]['yaw'])

        # This is a way to still have good gyro corrections if at least one
        # motor reaches its max
        '''
        maxMotor = np.max(motorvals)
        if (maxMotor > 1):
            motorvals = [motorvals[i] - (maxMotor-1) for i in range(m)]
        '''

        # Keep motor values in appropriate interval
        motorvals[motorvals < 0] = 0
        motorvals[motorvals > 1] = 1

        return motorvals.copy()


class QuadXAPMixer(Mixer):
    '''
    Mixer subclass for X-configuration quadcopters following the
    ArduPilot numbering convention:

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

    def __init__(self):

        Mixer.__init__(self, self.MOTORDIRS)


class CoaxialMixer(Mixer):

    MOTORDIRS = (
                 {'throttle': +1, 'roll': 0, 'pitch': 0, 'yaw': +1},
                 {'throttle': +1, 'roll': 0, 'pitch': 0, 'yaw': -1},
                 {'throttle': 0, 'roll': +1, 'pitch': 0, 'yaw': 0},
                 {'throttle': 0, 'roll': 0, 'pitch': +1, 'yaw': 0}
                )

    def __init__(self):

        Mixer.__init__(self, self.MOTORDIRS)
