'''
   Classes for PID controllers

   Copyright (c) 2021 Simon D. Levy

   MIT License
'''

import numpy as np

from demands import DEMANDS_ROLL, DEMANDS_PITCH, DEMANDS_YAW
from state import STATE_PHI, STATE_DPHI, STATE_THETA, STATE_DTHETA, STATE_DPSI
from debugging import debug


def _constrainAbs(val, lim):

    return -lim if val < -lim else (+lim if val > +lim else val)


class LevelPid:

    MAX_ANGLE_DEGREES = 45

    def __init__(self, Kp):

        self.Kp = Kp

        # Maximum roll pitch demand is +/-0.5, so to convert demand to
        # angle for error computation, we multiply by the folling amount:
        self.dmdscale = 2 * np.radians(self.MAX_ANGLE_DEGREES)

    def modifyDemands(self, state, demands):

        newdemands = demands.copy()

        newdemands[DEMANDS_ROLL] = self.Kp * (demands[DEMANDS_ROLL] *
                                              self.dmdscale -
                                              state[STATE_PHI])

        # Pitch demand is nose-down positive, so we negate
        # pitch-forward (nose-down negative) to reconcile them
        newdemands[DEMANDS_PITCH] = self.Kp * (demands[DEMANDS_PITCH] *
                                               self.dmdscale +
                                               state[STATE_THETA])

        # LevelPid uses no state
        return newdemands, None


class RatePid:

    # Arbitrary constants
    BIG_DEGREES_PER_SECOND = 40.0
    WINDUP_MAX = 6.0

    def __init__(self, Kp, Ki, Kd, windupMax=0.4):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.windupMax = windupMax

        # Convert degree parameters to radians for use later
        self.bigAngularVelocity = np.radians(self.BIG_DEGREES_PER_SECOND)

        # Initialize state
        self.state = [RatePid._newstate(), RatePid._newstate()]

    @staticmethod
    def _newstate():
        return {'deltaError1': 0,
                'deltaError2': 0,
                'errorI': 0,
                'lastError': 0}

    def modifyDemands(self, state, demands):

        newdemands = demands.copy()

        newdemands[DEMANDS_ROLL], self.state[0] = \
            RatePid._compute(self.Kp,
                             self.Ki,
                             self.Kd,
                             self.windupMax,
                             self.bigAngularVelocity,
                             demands,
                             DEMANDS_ROLL,
                             state,
                             STATE_DPHI,
                             self.state[0])

        newdemands[DEMANDS_PITCH], self.state[1] = \
            RatePid._compute(self.Kp,
                             self.Ki,
                             self.Kd,
                             self.windupMax,
                             self.bigAngularVelocity,
                             demands,
                             DEMANDS_PITCH,
                             state,
                             STATE_DTHETA,
                             self.state[1],
                             -1)

        return newdemands, self.state

    @staticmethod
    def _compute(Kp,
                 Ki,
                 Kd,
                 windupMax,
                 bigAngularVelocity,
                 demands,
                 demand_index,
                 vehicle_state,
                 vehicle_state_index,
                 controller_state,
                 state_direction=+1):
        '''
        Handles one degree of freedom of angular velocity.  We specify the
        direction of the velocity because pitch demand is postive for stick
        forward, but pitch angle is positive for nose up.
        '''

        angularVelocity = state_direction * vehicle_state[vehicle_state_index]

        # Reset integral on quick angular velocity change
        if abs(angularVelocity) > bigAngularVelocity:
            controller_state = RatePid._newstate()

        # Compute error as scaled target minus actual
        error = demands[demand_index] - angularVelocity

        # Compute P term
        pterm = error * Kp

        # Compute I term
        errorI = _constrainAbs(controller_state['errorI'] + error,
                               windupMax)
        iterm = errorI * Ki

        # Compute D term
        deltaError = error - controller_state['lastError']
        dterm = ((controller_state['deltaError1'] +
                  controller_state['deltaError2'] + deltaError) * Kd)

        # Update controller state
        controller_state['deltaErrro2'] = controller_state['deltaError1']
        controller_state['deltaErrro1'] = deltaError
        controller_state['errorI'] = errorI
        controller_state['lasteError'] = error

        return pterm + iterm + dterm, controller_state


class YawPid:

    def __init__(self, Kp, Ki, windupMax=0.4):

        self.Kp = Kp
        self.Ki = Ki
        self.windupMax = windupMax

        self.state = {'errorI': 0, 'lastError': 0}

    def modifyDemands(self, state, demands):

        # Compute error as scaled target minus actual
        error = demands[DEMANDS_YAW] - state[STATE_DPSI]

        # Accumualte error integral
        errorI = _constrainAbs(self.state['errorI'] + error, self.windupMax)

        # Update controller state
        self.state = {'errorI': errorI, 'lastError': error}

        # Update demands
        newdemands = demands.copy()
        newdemands[DEMANDS_YAW] = error * self.Kp + errorI * self.Ki

        return newdemands, self.state
