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

class MinimalControl : public hf::Control {

    public:

        void run(const float dt, const hf::state_t & state, hf::demands_t & demands) 
        {
            const auto resetPids = demands.thrust < THROTTLE_DOWN;

            _pitchRollAnglePid.run(dt, resetPids, state, demands);

            _pitchRollRatePid.run(dt, resetPids, state, demands);

            _yawRatePid.run(dt, resetPids, state, demands);
        }

    private:

        static constexpr float THROTTLE_DOWN = 0.06;

        hf::YawRatePid _yawRatePid;

        hf::PitchRollAnglePid _pitchRollAnglePid;

        hf::PitchRollRatePid _pitchRollRatePid;
};

void setup() 
{
    _board.init();
}

void loop() 
{
    static MinimalControl _control;

    _board.step(&_control);
}
