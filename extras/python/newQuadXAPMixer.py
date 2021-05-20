"""
   Mixer subclass for X-configuration quadcopters following the
   ArduPilot numbering convention:
    3cw   1ccw
       | /
        ^
       / |
    2ccw  4cw

Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy
MIT License
"""

from demands import DEMANDS_THROTTLE, DEMANDS_PITCH, DEMANDS_ROLL, DEMANDS_YAW
import numpy as np
# from debugging import debug


# Replicating the motormixer struct from the C++ program
class MotorMixer(object):

    def __init__(self, throttle=0, roll=0, pitch=0, yaw=0):

        self.throttle = throttle
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


class Mixer(object):
    MAXMOTORS = 20

    def __init__(self):
        # Create a list with 4 MotorMixer objects
        self.motordirs = []
        for i in range(4):
            self.motordirs.append(MotorMixer())

        # Reference for the settings below
        #                       Th  RR  PF  YR
        # self.motordirs[0] = { +1, -1, -1, +1 }    # 1 right front
        # self.motordirs[1] = { +1, +1, +1, +1 }    # 2 left rear
        # self.motordirs[2] = { +1, +1, -1, -1 }    # 3 left front
        # self.motordirs[3] = { +1, -1, +1, -1 }    # 4 right rear

        # Manually set the values of the MotorMixer objects
        # for a quadcopter in AP configuration
        self.motordirs[0].throttle = 1
        self.motordirs[0].roll = -1
        self.motordirs[0].pitch = -1
        self.motordirs[0].yaw = 1

        self.motordirs[1].throttle = 1
        self.motordirs[1].roll = 1
        self.motordirs[1].pitch = 1
        self.motordirs[1].yaw = 1

        self.motordirs[2].throttle = 1
        self.motordirs[2].roll = 1
        self.motordirs[2].pitch = -1
        self.motordirs[2].yaw = -1

        self.motordirs[3].throttle = 1
        self.motordirs[3].roll = -1
        self.motordirs[3].pitch = 1
        self.motordirs[3].yaw = -1

    def run(self, demands):
        """
        Turn demands into motor spins in [0,1]
        """
        # return 0.6 * np.ones(4)

        # XXX to fix
        # Supposed to lock the throttle values between [0,1]
        # But it just tends to make things messy
        # demands[DEMANDS_THROTTLE] = (demands[DEMANDS_THROTTLE]+1)/2

        # Create a vector to be returned
        motorvals = np.zeros(4)

        for i in range(4):  # motorvals[i]

            motorvals[i] = (demands[DEMANDS_THROTTLE] *
                            self.motordirs[i].throttle +

                            demands[DEMANDS_ROLL] *
                            self.motordirs[i].roll +

                            demands[DEMANDS_PITCH] *
                            self.motordirs[i].pitch +

                            demands[DEMANDS_YAW] *
                            self.motordirs[i].yaw)

        # XXX Something goes wrong in here.
        # That makes the motorvals into Nan or somethings

        return motorvals
        """
        maxMotor = motorvals[0]

        for i in range(1, 4):
            if (motorvals[i] > maxMotor):
                maxMotor = motorvals[i]

        for i in range(4):
            # This is a way to still have good gyro corrections
            # if at least one motor reaches its max
            if (maxMotor > 1):
                motorvals[i] -= maxMotor - 1
            # Keep motor values in appropriate interval
            motorvals[i] = self.constrainMinMax(motorvals[i], 0, 1)

        """
    def constrainMinMax(self, val, min, max):
        if val < min:
            return min
        elif val > max:
            return max
