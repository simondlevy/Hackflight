'''
   Classes for PID controllers

   Copyright (c) 2021 Simon D. Levy

   MIT License
'''

import numpy as np

from demands import DEMANDS_ROLL, DEMANDS_PITCH, DEMANDS_YAW
from state import STATE_PHI, STATE_DPHI, STATE_THETA, STATE_DTHETA, STATE_DPSI


def _constrainAbs(val, lim):

    return -lim if val < -lim else (+lim if val > +lim else val)


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

        new_demands = demands.copy()

        new_demands[DEMANDS_ROLL], self.state[0] = \
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

        new_demands[DEMANDS_PITCH], self.state[1] = \
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

        return new_demands, self.state

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


def make_yaw_pid(Kp, Ki, windupMax=0.4):
    '''
    A closure for rate control of yaw angle
    '''

    initial_state = {'errorI': 0, 'lastError': 0}

    def apply(vehicle_state, controller_state, demands):

        # Compute error as scaled target minus actual
        error = demands[DEMANDS_YAW] - vehicle_state[STATE_DPSI]

        # Accumualte error integral
        errorI = _constrainAbs(controller_state['errorI'] + error, windupMax)

        # Update controller state
        new_controller_state = {'errorI': errorI, 'lastError': error}

        # Update demands
        new_demands = demands.copy()
        new_demands[DEMANDS_YAW] = error * Kp + errorI * Ki

        return new_demands, new_controller_state

    return apply, initial_state


def make_level_pid(Kp):

    MAX_ANGLE_DEGREES = 45

    # Maximum roll pitch demand is +/-0.5, so to convert demand to
    # angle for error computation, we multiply by the folling amount:
    dmdscale = 2 * np.radians(MAX_ANGLE_DEGREES)

    def apply(vehicle_state, controller_state, demands):

        new_demands = demands.copy()

        new_demands[DEMANDS_ROLL] = Kp * (demands[DEMANDS_ROLL] * dmdscale -
                                          vehicle_state[STATE_PHI])

        # Pitch demand is nose-down positive, so we negate
        # pitch-forward (nose-down negative) to reconcile them
        new_demands[DEMANDS_PITCH] = Kp * (demands[DEMANDS_PITCH] * dmdscale +
                                           vehicle_state[STATE_THETA])

        # LevelPid uses no controller state
        return new_demands, None

    return apply, None
