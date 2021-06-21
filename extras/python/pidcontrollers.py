'''
   Classes for PID controllers

   Copyright (c) 2021 Simon D. Levy

   MIT License
'''

from closedloop import ClosedLoopController
from demands import DEMANDS_ROLL, DEMANDS_PITCH, DEMANDS_YAW
from state import STATE_PHI, STATE_DPHI, STATE_THETA, STATE_DTHETA, STATE_DPSI
import numpy as np


class _DofPid:
    '''
    PID controller for a single degree of freedom.  Because time differences
    (dt) appear more-or-less constant, we avoid incoroporating them into the
    code i.e., they are "absorbed" into tuning constants Ki and Kd.
    '''

    def __init__(self, Kp, Ki, Kd, windupMax=0.4):

        # Constants
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.windupMax = windupMax

        # Accumulated values
        self.lastError = 0
        self.errorI = 0
        self.deltaError1 = 0
        self.deltaError2 = 0

        # For deltaT-based controllers
        self.previousTime = 0

        # Initialize error integral, previous value
        self.reset()

    def compute(self, target, actual):

        # Compute error as scaled target minus actual
        error = target - actual

        # Compute P term
        pterm = error * self.Kp

        # Compute I term
        iterm = 0
        if self.Ki > 0:  # optimization
            self.errorI = _DofPid.constrainAbs(self.errorI + error,
                                               self.windupMax)
            iterm = self.errorI * self.Ki

        # Compute D term
        dterm = 0
        if self.Kd > 0:  # optimization
            deltaError = error - self.lastError
            dterm = ((self.deltaError1 + self.deltaError2 + deltaError) *
                     self.Kd)
            self.deltaError2 = self.deltaError1
            self.deltaError1 = deltaError
            self.lastError = error

        return pterm + iterm + dterm

    def reset(self):

        self.errorI = 0
        self.lastError = 0
        self.previousTime = 0

    @staticmethod
    def constrainAbs(val, lim):

        return -lim if val < -lim else (+lim if val > +lim else val)


class _AnglePid(_DofPid):

    MAX_ANGLE_DEGREES = 45

    def __init__(self, Kp):

        _DofPid.__init__(self, Kp, 0, 0)

        # Maximum roll pitch demand is +/-0.5, so to convert demand to
        # angle for error computation, we multiply by the folling amount:
        self.demandMultiplier = 2 * np.radians(self.MAX_ANGLE_DEGREES)

    def compute(self, demand, angle):

        return _DofPid.compute(self, demand * self.demandMultiplier, angle)


class _AngularVelocityPid(_DofPid):

    # Arbitrary constants
    BIG_DEGREES_PER_SECOND = 40.0
    WINDUP_MAX = 6.0

    def __init__(self, Kp, Ki, Kd):

        _DofPid.__init__(self, Kp, Ki, Kd, self.WINDUP_MAX)

        # Convert degree parameters to radians for use later
        self.bigAngularVelocity = np.radians(self.BIG_DEGREES_PER_SECOND)

    def compute(self, demand, angularVelocity):

        # Reset integral on quick angular velocity change
        if abs(angularVelocity) > self.bigAngularVelocity:
            self.reset()

        return _DofPid.compute(self, demand, angularVelocity)


class RatePid(ClosedLoopController):

    def __init__(self, Kp, Ki, Kd):

        self.rollPid = _AngularVelocityPid(Kp, Ki, Kd)
        self.pitchPid = _AngularVelocityPid(Kp, Ki, Kd)

    def modifyDemands(self, state, demands):

        newdmnds = demands.copy()

        # Roll angle and roll demand are both positive for starboard right down
        newdmnds[DEMANDS_ROLL] = self.rollPid.compute(demands[DEMANDS_ROLL],
                                                      state[STATE_DPHI])

        # Pitch demand is postive for stick forward, but pitch angle is
        # positive for nose up.  So we negate pitch angle to compute demand
        newdmnds[DEMANDS_PITCH] = self.pitchPid.compute(demands[DEMANDS_PITCH],
                                                        -state[STATE_DTHETA])

        return newdmnds


class YawPid(ClosedLoopController):

    def __init__(self, Kp, Ki):

        self.yawPid = _AngularVelocityPid(Kp, Ki, 0)

    def modifyDemands(self, state, demands):

        newdmnds = demands.copy()

        newdmnds[DEMANDS_YAW] = self.yawPid.compute(demands[DEMANDS_YAW],
                                                    state[STATE_DPSI])

        return newdmnds


class LevelPid(ClosedLoopController):

    def __init__(self, Kp):

        self.rollPid = _AnglePid(Kp)
        self.pitchPid = _AnglePid(Kp)

    def modifyDemands(self, state, demands):

        newdmnds = demands.copy()

        # Roll angle and roll demand are both positive for starboard right down
        newdmnds[DEMANDS_ROLL] = self.rollPid.compute(demands[DEMANDS_ROLL],
                                                      state[STATE_PHI])

        # Pitch demand is postive for stick forward, but pitch angle is
        # positive for nose up.  So we negate pitch angle to compute demand
        newdmnds[DEMANDS_PITCH] = self.pitchPid.compute(demands[DEMANDS_PITCH],
                                                        -state[STATE_THETA])

        return newdmnds
