/* 
 *  Ping-pong autopilot using 1x1 rangefinder
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
#include <simulator/pose.h>
#include <autopilot/rangefinder.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

#include "autopilot.hpp"

class PingPongAutopilot : public Autopilot {

    private:

        simsens::Rangefinder * _rangefinderForward;

    public:

        void init(simsens::Robot & robot)
        {
            _rangefinderForward = robot.rangefinders["VL53L1-forward"];
        }

        void getSetpoint(hf::demands_t & setpoint)
        {
            setpoint.yaw = 1.0;
        }

        void readSensors(simsens::World & world, const hf::pose_t & pose,
                FILE * logfile)
        {
            (void)world;
            (void)pose;
            (void)logfile;
        }
};
