/*
   Hackflight example with C++ PID controllers

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

#include <mixers.hpp>

static hf::TurtleBoard _board;

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
    float phi=0, theta=0, psi=0, dphi=0, dtheta=0, dpsi=0;

    _board.readData(dt, 
            thrustDemand, rollDemand, pitchDemand, yawDemand,
            phi, theta, psi, dphi, dtheta, dpsi);

    const auto resetPids = thrustDemand < THROTTLE_DOWN;

    _pitchRollAnglePid.run(
            dt, resetPids, rollDemand, pitchDemand, phi, theta);

    _pitchRollRatePid.run(
            dt, resetPids, rollDemand, pitchDemand, dphi, dtheta);

    _yawRatePid.run(dt, resetPids, yawDemand, dpsi);

    float m1_command=0, m2_command=0, m3_command=0, m4_command=0;

    hf::Mixer::runBetaFlightQuadX(
            thrustDemand, rollDemand, pitchDemand, yawDemand, 
            m1_command, m2_command, m3_command, m4_command);

    _board.runMotors(m1_command, m2_command, m3_command, m4_command);
}
