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
        //simsens::Rangefinder * _rangefinderBackward;

        int readRangefinder(simsens::Rangefinder * rangefinder,
                simsens::World & world, const hf::pose_t & pose)
        {
            int distance_mm = 0;

            rangefinder->read(
                    {pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi},
                    world, &distance_mm);

            return distance_mm;
        }

    public:

        void init(simsens::Robot & robot)
        {
            _rangefinderForward = robot.rangefinders["VL53L1-forward"];
            //_rangefinderBackward = robot.rangefinders["VL53L1-backward"];
        }

        void getSetpoint(hf::demands_t & setpoint)
        {
            (void)setpoint;
        }

        void readSensors(simsens::World & world, const hf::pose_t & pose,
                FILE * logfile)
        {
            printf("%d\n", readRangefinder(_rangefinderForward, world, pose));

            (void)logfile;

        }
};
