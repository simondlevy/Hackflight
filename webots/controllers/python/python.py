'''
  Hackflight flight simulator using Webots
 
  Copyright (C) 2024 Simon D. Levy, Tiffany Guo, James Xia
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http:--www.gnu.org/licenses/>.
'''

from time import time

from controller import Robot, InertialUnit, Gyro, GPS

from sticks import Sticks

from pids import (altitude, climb_rate, pitch_roll_angle, 
                  pitch_roll_rate, position, yaw_angle, yaw_rate)

from mixers import runcf
from vehicle_state import VehicleState

# PID control constants
PITCH_ROLL_ANGLE_KP = 6e0
PITCH_ROLL_RATE_KP = 1.25e-2
PITCH_ROLL_RATE_KD = 1.25e-4
YAW_RATE_KP = 1.20e-2

# Motor thrust constants for climb-rate PID controller
TBASE = 56
TSCALE = 1.0
TMIN = 0

# https://www.bitcraze.io/documentation/tutorials/getting-started-with-flow-deck/
ALTITUDE_TARGET_MIN = 0.2
ALTITUDE_TARGET_MAX = 2.0  # 3.0 in original
ALTITUDE_TARGET_INITIAL = 0.2

STATUS_LANDED = 0
STATUS_TAKING_OFF = 1
STATUS_FLYING = 2

# We consider throttle inprint above this below this value to be
# positive for takeoff
THROTTLE_ZERO = 0.05

THROTTLE_SCALE = 0.005

# We consider altitudes below this value to be the ground
ZGROUND = 0.05


# Arbitrary time constant
DT = .01

LOGFILE = '../playback-pose/pose.csv'


def makeMotor(robot, name, direction):

    motor = robot.getDevice(name)

    motor.setPosition(float('+inf'))
    motor.setVelocity(direction)

    return motor


def main():

    print('- Use W and S to go up and down\n');
    print('- Use arrow keys to move in the horizontal plane\n');
    print('- Use Q and E to rotate around yaw\n');

    logfile = open(LOGFILE, 'w')

    status = STATUS_LANDED
    altitude_target = 0

    positionController = position.PositionController()

    pitchRollAngleController = pitch_roll_angle.PitchRollAngleController()

    pitchRollRateController = pitch_roll_rate.PitchRollRateController()

    altitudeController = altitude.AltitudeController()

    climbRateController = climb_rate.ClimbRateController()

    yawAngleController = yaw_angle.YawAngleController()

    yawRateController = yaw_rate.YawRateController()

    robot = Robot()

    m1 = makeMotor(robot, 'motor1', +1)
    m2 = makeMotor(robot, 'motor2', -1)
    m3 = makeMotor(robot, 'motor3', +1)
    m4 = makeMotor(robot, 'motor4', -1)

    timestep = int(robot.getBasicTimeStep())

    imu = InertialUnit('inertial_unit')
    imu.enable(timestep)

    gyro = Gyro('gyro')
    gyro.enable(timestep)

    gps = GPS('gps')
    gps.enable(timestep)

    state = VehicleState()

    sticks = Sticks(timestep)

    start_time = time()

    while robot.step(timestep) != -1:

        # Get open-loop demands from input device (keyboard, joystick, etc.)
        demands = list(sticks.getDemands())

        debug = False

        # "Autopilot" takeoff / maneuver for debugging
        curr_time = time() - start_time
        if False:
            if curr_time < 2:
                demands[0] = +1
            if curr_time > 6 and curr_time < 7:
                demands[3] = -1
                debug = True
                #print('%+3.3f' % state.ang.z)

        # Get vehicle state from sensors
        state.get(robot, gyro, imu, gps)

        # A simple state machine for flying status

        if status == STATUS_TAKING_OFF:

            status = (STATUS_FLYING if state.pos.z > ZGROUND else status)

        elif status == STATUS_FLYING:

            status = (STATUS_LANDED
                      if state.pos.z <= ZGROUND
                      else status)

            altitude_target += THROTTLE_SCALE * demands[0]

        else:  # LANDED
            status = (STATUS_TAKING_OFF
                      if demands[0] > THROTTLE_ZERO
                      else status)

            altitude_target = ALTITUDE_TARGET_INITIAL

        landed = status == STATUS_LANDED

        # Run PID controllers on open-loop demands and vehicle state to get
        # closed-loop demands
        demands = positionController.run(
                state.dpos.x, state.dpos.y, DT, demands)

        demands = pitchRollAngleController.run(
                PITCH_ROLL_ANGLE_KP, state.ang.x, state.ang.y, DT, demands)

        demands = pitchRollRateController.run(
                PITCH_ROLL_RATE_KP, PITCH_ROLL_RATE_KD,
                state.dang.x, state.dang.y, DT, landed, demands)

        demands = altitudeController.run(
                state.pos.z, DT, altitude_target, demands)

        demands = climbRateController.run(
                state.dpos.z, DT, TBASE, TSCALE, TMIN, not landed, demands)

        demands = yawAngleController.run(state.ang.z, DT, demands)

        demands = yawRateController.run(YAW_RATE_KP, state.dang.z, DT,  demands)

        m1val, m2val, m3val, m4val = runcf(demands)

        m1.setVelocity(+m1val)
        m2.setVelocity(-m2val)
        m3.setVelocity(+m3val)
        m4.setVelocity(-m4val)

        # Log the pose and motor spins
        if not landed:
            logfile.write('%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n' %
                          (state.pos.x, state.pos.y, state.pos.z, 
                           state.ang.x, state.ang.y, -state.ang.z,
                           m1val, m2val, m3val, m4val))
            logfile.flush()




main()
