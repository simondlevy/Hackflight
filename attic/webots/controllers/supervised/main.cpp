/* 
   C++ flight simulator kinematic program 

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

#include <sim/sim2.hpp>
#include <pids/altitude.hpp>
#include <pids/position.hpp>
#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>
#include <pids/yaw_rate.hpp>
#include <mixers/bfquadx.hpp>

static const float PITCH_ROLL_POST_SCALE = 50;

static hf::Simulator _sim;

namespace hf {

    static AltitudePid _altitudePid;

    static YawRatePid _yawRatePid;

    static PitchRollAnglePid _pitchRollAnglePid;

    static PitchRollRatePid _pitchRollRatePid;

    void run_closed_loop_controllers(
            float dt, const state_t & state, demands_t & demands)
    {
        dt = 0.01;

        _altitudePid.run(_sim.isSpringy(), dt, state, demands);

        PositionPid::run(state, demands);

        _pitchRollAnglePid.run(dt, false, state, demands);

        _pitchRollRatePid.run(dt, false, state, demands,
                PITCH_ROLL_POST_SCALE);

        _yawRatePid.run(dt, false, state, demands);
    }
}

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    hf::BfQuadXMixer mixer = {};

    _sim.init(mixer);

    while (true) {

        if (!_sim.step()) {
            break;
        }
    }

    _sim.close();

    return 0;
}
