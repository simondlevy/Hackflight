'''
   Closures for PID controllers

   Copyright (c) 2021 Simon D. Levy

   MIT License
'''

import numpy as np

from demands import DEMANDS_THROTTLE, DEMANDS_ROLL, DEMANDS_PITCH, DEMANDS_YAW

from state import STATE_Z, STATE_DZ, STATE_PHI, STATE_DPHI, STATE_THETA, \
                  STATE_DTHETA, STATE_DPSI

from debugging import debug


def _constrainAbs(val, lim):

    return -lim if val < -lim else (+lim if val > +lim else val)


# Rate -----------------------------------------------------------------------


def rate_pid(Kp=0.225, Ki=0.001875, Kd=0.375,
             windupMax=0.4, maxDegreesPerSecond=40):
    '''
    A closure for PID control of pitch and roll angular velocities
    '''

    # Helper
    def newstate():
        return {'deltaError1': 0,
                'deltaError2': 0,
                'errorI': 0,
                'lastError': 0}

    # Helper
    def compute(Kp, Ki, Kd, windupMax, maxvel, demands, demand_index,
                vehicle_state, vehicle_state_index, controller_state,
                state_direction=+1):
        '''
        Handles one degree of freedom of angular velocity.  We specify the
        direction of the velocity because pitch demand is postive for stick
        forward, but pitch angle is positive for nose up.
        '''

        angvel = state_direction * vehicle_state[vehicle_state_index]

        # Reset integral on quick angular velocity change
        if abs(angvel) > maxvel:
            controller_state = newstate()

        # Compute error as target minus actual
        error = demands[demand_index] - angvel

        # Compute P term
        pterm = error * Kp

        # Compute I term
        errorI = _constrainAbs(controller_state['errorI'] + error, windupMax)
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

    # Convert degree parameters to radians for use later
    maxvel = np.radians(maxDegreesPerSecond)

    # Initialize controller state
    initial_controller_state = newstate(), newstate()

    def apply(vehicle_state, controller_state, demands):

        new_demands = demands.copy()

        new_demands[DEMANDS_ROLL], new_roll_controller_state = \
            compute(Kp, Ki, Kd, windupMax, maxvel, demands, DEMANDS_ROLL,
                    vehicle_state, STATE_DPHI, controller_state[0])

        new_demands[DEMANDS_PITCH], new_pitch_controller_state = \
            compute(Kp, Ki, Kd, windupMax, maxvel, demands, DEMANDS_PITCH,
                    vehicle_state, STATE_DTHETA, controller_state[1], -1)

        return (new_demands,
                (new_roll_controller_state, new_pitch_controller_state))

    return apply, initial_controller_state


# Yaw ------------------------------------------------------------------------


def yaw_pid(Kp=2.0, Ki=0.1, windupMax=0.4):
    '''
    A closure for PI control of yaw angular velocity
    '''

    initial_controller_state = {'errorI': 0}

    def apply(vehicle_state, controller_state, demands):

        # Compute error as target minus actual
        error = demands[DEMANDS_YAW] - vehicle_state[STATE_DPSI]

        # Accumualte error integral
        errorI = _constrainAbs(controller_state['errorI'] + error, windupMax)

        # Update controller state
        new_controller_state = {'errorI': errorI}

        # Update demands
        new_demands = demands.copy()
        new_demands[DEMANDS_YAW] = error * Kp + errorI * Ki

        return new_demands, new_controller_state

    return apply, initial_controller_state


# Level -----------------------------------------------------------------------

def level_pid(Kp=0.2, maxAngleDegrees=45):
    '''
    A closure for P control of pitch and roll angles
    '''

    # Maximum roll pitch demand is +/-0.5, so to convert demand to
    # angle for error computation, we multiply by the folling amount:
    dmdscale = 2 * np.radians(maxAngleDegrees)

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


# Altitude hold ---------------------------------------------------------------

def alt_hold_pid(Kp=0.75, Ki=1.5, windupMax=0.4,
                 pilotVelZMax=2.5, stickDeadband=0.2):
    '''
    A closure for PI control of altitude
    '''

    initial_controller_state = {'errorI': 0,
                                'targetAltitude': 0,
                                'inBand': False}

    def apply(vehicle_state, controller_state, demands):

        errorI = controller_state['errorI']
        targetAltitude = controller_state['targetAltitude']

        # NED => ENU
        altitude = -vehicle_state[STATE_Z]

        throttleDemand = demands[DEMANDS_THROTTLE]

        debug(throttleDemand)

        # Is stick demand in deadband?
        inBand = abs(throttleDemand) < stickDeadband

        # Reset controller when moving into deadband
        if inBand and not controller_state['inBand']:
            errorI = 0
            targetAltitude = altitude

        # Target velocity is a setpoint inside deadband, scaled
        # constant outside
        targetVelocity = (targetAltitude - altitude
                          if inBand
                          else pilotVelZMax * throttleDemand)

        # Compute error as target velocity minus actual velocity, after
        # negating actual to accommodate NED
        error = targetVelocity + vehicle_state[STATE_DZ]

        # Accumualte error integral
        errorI = _constrainAbs(controller_state['errorI'] + error, windupMax)

        # Update controller state
        new_controller_state = {'errorI': errorI,
                                'targetAltitude': targetAltitude,
                                'inBand': inBand}

        # Update demands
        new_demands = demands.copy()
        new_demands[DEMANDS_THROTTLE] = error * Kp + errorI * Ki

        return new_demands, new_controller_state

    return apply, initial_controller_state
