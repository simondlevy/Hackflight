"""
   Mixer subclass for X-configuration quadcopters following the
   ArduPilot numbering convention:
    3cw   1ccw
       | /
        ^
       /  |
    2ccw  4cw
Copyright (C) 2021 S.Basnet, N. Manaye, N. Nguyen, S.D. Levy

MIT License
"""


from mixer import Mixer
from debugging import debug


class QuadXAPMixer(object):

    def __init__(self):
        motorDirections = []
        for i in range(4):
            motorDirections.append(MotorMixer())

        motorDirections[0] = { +1, -1, -1, +1 }    # 1 right front
        motorDirections[1] = { +1, +1, +1, +1 }    # 2 left rear
        motorDirections[2] = { +1, +1, -1, -1 }    # 3 left front
        motorDirections[3] = { +1, -1, +1, -1 }    # 4 right rear

    def getOmega(self, demands):
        """
        Turns demands U to motor spins Omega
        """

        u = demands
        omega = self.run(u)

        debug(omega)
        return omega

# Replicating the motormixer struct from the C++ program
class MotorMixer(object):

    def __init__(self, throttle=0, roll=0, pitch=0, yaw=0):

        self.throttle = throttle
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        omega = Mixer.run(u)
        return omega
