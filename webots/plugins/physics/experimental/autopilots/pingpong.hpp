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

        simsens::Rangefinder * _rangefinderTmp;
        simsens::Rangefinder * _rangefinderForward;
        //simsens::Rangefinder * _rangefinderBackward;

        void readRangefinder(simsens::Rangefinder * rangefinder,
                simsens::World & world, const hf::pose_t & pose,
                const int size)
        {
            int d[1000] = {};
            rangefinder->read(
                    {pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi},
                    world, d);
            for (int i=0; i<size; ++i) {
                printf("%d ", d[i]);
            }
            printf("\n-------------------------\n");
        }

    public:

        void init(simsens::Robot & robot)
        {
            _rangefinderTmp = robot.rangefinders["VL53L1-tmp"];
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
            readRangefinder(_rangefinderTmp, world, pose, 1);
            readRangefinder(_rangefinderForward, world, pose, 1);

            (void)logfile;

        }
};
