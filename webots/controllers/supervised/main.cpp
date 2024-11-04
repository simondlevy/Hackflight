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
#include <pids/yaw_rate.hpp>
#include <mixers/bfquadx.hpp>

static const float YAW_PRESCALE = 160; // deg/sec

static const float THRUST_BASE = 55.385;

static const float THROTTLE_DOWN = 0.06;

static const float THROTTLE_DEADBAND = 0.2;

static const float PITCH_ROLL_POST_SCALE = 50;

static bool _reset_pids;

static hf::demands_t _open_loop_demands;

static hf::Simulator _sim;


namespace hf {

    static AltitudePid _altitudePid;

    static YawRatePid _yawRatePid;


    void run_closed_loop_controllers(
            const float dt, const state_t & state, demands_t & demands)
    {
        _altitudePid.run(_sim.isSpringy(), dt, state, _open_loop_demands, demands);

        _yawRatePid.run(dt, _reset_pids, state, demands);
    }
}

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    hf::BfQuadXMixer mixer = {};

    _sim.init(mixer);

    hf::demands_t demands = {};

    while (true) {

        if (!_sim.step(demands)) {
            break;
        }

        _open_loop_demands = _sim.getDemands();

        demands.yaw = _open_loop_demands.yaw * YAW_PRESCALE;

        _reset_pids = demands.thrust < THROTTLE_DOWN;

        // Throttle control begins when once takeoff is requested, either by
        // hitting a button or key ("springy", self-centering throttle) or by
        // raising the non-self-centering throttle stick
        if (_sim.requestedTakeoff()) {

        }    
    }

    _sim.close();

    return 0;
}
