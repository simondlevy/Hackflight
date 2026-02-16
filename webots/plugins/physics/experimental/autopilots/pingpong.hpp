/* 
 *  "Ping-Pong" autopilot using 1x1 rangefinder
 *
 *  Copyright (C) 2026 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 *  This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

// Hackflight
#include <simulator/dynamics.hpp>
#include <autopilots/pingpong.hpp>

#include "autopilot.hpp"

class PingPongAutopilot : public Autopilot {

    private:

        hf::PingPongAutopilot _helper;

    public:

        void getSetpoint(const hf::Dynamics::state_t state, hf::demands_t & setpoint) 
        {
            _helper.getSetpoint(state, setpoint);
        }

        void readSensors(
                simsens::Robot & robot,
                simsens::World & world,
                const hf::Dynamics::state_t & state,
                FILE * logfile)
        {
            (void)logfile;

            _helper.readSensors(robot, world, state);
        }
};
