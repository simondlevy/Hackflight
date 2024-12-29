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

#include <hackflight.hpp>

#include <board_tinypico.hpp>

#include <control.hpp>

#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>
#include <pids/yaw_rate.hpp>

static hf::Board _board;

static constexpr float THROTTLE_DOWN = 0.06;

static hf::YawRatePid _yawRatePid;

static hf::PitchRollAnglePid _pitchRollAnglePid;
static hf::PitchRollRatePid _pitchRollRatePid;

static hf::BfQuadXMixer _mixer;

void setup() 
{
    _board.init();
}

void loop() 
{
    float dt=0;
    hf::demands_t demands = {};
    hf::state_t state = {};

    _board.readData(dt, demands, state);

    const auto resetPids = demands.thrust < THROTTLE_DOWN;

    _pitchRollAnglePid.run(dt, resetPids, state, demands);

    _pitchRollRatePid.run(dt, resetPids, state, demands);

    _yawRatePid.run(dt, resetPids, state, demands);

    static uint32_t _msec_prev;
    const auto msec_curr = millis();
    if (msec_curr - _msec_prev > 20) {
        printf("t=%3.3f  r=%+3.3f  p=%+3.3f  y=%+3.3f\n",
                demands.thrust,
                demands.roll,
                demands.pitch,
                demands.yaw);
        _msec_prev = msec_curr;
    }

    float motors[4] = {};

    _mixer.run(demands, motors);

    _board.runMotors(motors);
}
