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

#include <hackflight.h>

#include <teensy/board.hpp>

#include <teensy/mixers/bfquadx.hpp>

#include <teensy/pids/pitch_roll_angle.hpp>
#include <teensy/pids/pitch_roll_rate.hpp>
#include <teensy/pids/yaw_rate.hpp>

static Board _board;

static constexpr float THROTTLE_DOWN = 0.06;

static YawRatePid _yawRatePid;

static PitchRollAnglePid _pitchRollAnglePid;
static PitchRollRatePid _pitchRollRatePid;

static BfQuadXMixer _mixer;

void setup() 
{
    _board.init();
}

void loop() 
{
    float dt=0;
    demands_t demands = {};
    state_t state = {};

    _board.readData(dt, demands, state);

/*
    const auto s = state;
    printf("phi=%+3.3f dphi=%+3.3f theta=%+3.3f dtheta=%+3.3f "
            "psi = %+3.3f dpsi=%+3.3f |  ",
            s.phi, s.dphi, s.theta, s.dtheta, s.psi, s.dpsi);
    printf("roll=%+3.3f pitch=%+3.3f yaw = %+3.3f\n",
            demands.roll, demands.pitch, demands.yaw);
*/

    const auto resetPids = demands.thrust < THROTTLE_DOWN;

    _pitchRollAnglePid.run(dt, resetPids, state, demands);

    _pitchRollRatePid.run(dt, resetPids, state, demands);

    _yawRatePid.run(dt, resetPids, state, demands);

    float motors[4] = {};

    _mixer.run(demands, motors);

    _board.runMotors(motors);
}
