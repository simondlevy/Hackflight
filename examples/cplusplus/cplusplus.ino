/*
  Hackflight example sketch custom QuadX frame with Spektrum DSMX receiver

  Adapted from https://github.com/nickrehm/dRehmFlight
 
  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <turtle_board.hpp>

#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>
#include <pids/yaw_rate.hpp>

static hf::TurtleBoard _board;

// PID control ---------------------------------------------------------------

static constexpr float THROTTLE_DOWN = 0.06;

static hf::YawRatePid _yawRatePid;

static hf::PitchRollAnglePid _pitchRollAnglePid;
static hf::PitchRollRatePid _pitchRollRatePid;


void setup() 
{
    _board.init();
}

void loop() 
{
    float dt=0;
    float thrustDemand=0, rollDemand=0, pitchDemand=0, yawDemand=0;
    float phi=0, theta=0, psi=0, gyroX=0, gyroY=0, gyroZ=0;

    _board.readData(dt, 
            thrustDemand, rollDemand, pitchDemand, yawDemand,
            phi, theta, psi, gyroX, gyroY, gyroZ);

    const auto resetPids = thrustDemand < THROTTLE_DOWN;

    // Run demands through PID controllers

    _pitchRollAnglePid.run(
            dt, resetPids, rollDemand, pitchDemand, phi, theta);

    _pitchRollRatePid.run(
            dt, resetPids, rollDemand, pitchDemand, gyroX, gyroY);

    _yawRatePid.run(dt, resetPids, yawDemand, gyroZ);

    _board.runMixer( thrustDemand, rollDemand, pitchDemand, yawDemand);
}
