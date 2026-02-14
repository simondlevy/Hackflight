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

class SingleBeamRangefinder {

    friend class PingPongAutopilot;

    private:

        simsens::Rangefinder * rangefinder;
        
        int distance_mm;

        void init(simsens::Robot & robot, const string name)
        {
            rangefinder = robot.rangefinders[name];
        }

        void read(simsens::World & world, const hf::pose_t & pose)
        {
            rangefinder->read(
                    {pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi},
                    world, &distance_mm);
         }

};

class PingPongAutopilot : public Autopilot {

    private:

        SingleBeamRangefinder  _rangefinderForward;
        SingleBeamRangefinder  _rangefinderBackward;

    public:

        void init(simsens::Robot & robot)
        {
            _rangefinderForward.init(robot, "VL53L1-forward");
            _rangefinderBackward.init(robot, "VL53L1-backward");
        }

        void getSetpoint(hf::demands_t & setpoint)
        {
            (void)setpoint;
        }

        void readSensors(simsens::World & world, const hf::pose_t & pose,
                FILE * logfile)
        {
            _rangefinderForward.read(world, pose);
            _rangefinderBackward.read(world, pose);

            printf("forward=%4dmm  backward=%4dmm\n",
                    _rangefinderForward.distance_mm,
                    _rangefinderBackward.distance_mm);

            (void)logfile;

        }
};
