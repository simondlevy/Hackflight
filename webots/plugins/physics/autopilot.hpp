/* 
 * Webots autopilot custom physics plugin support for Hackflight
 *
 *  Copyright (C) 2025 Simon D. Levy
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

#include <stdio.h>

#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/world.hpp>

#include "helper.hpp"

class AutopilotHelper {

    private:

        static constexpr char PATH_VARIABLE_NAME[] = "WEBOTS_PATH";

        FILE * _logfile;

        PluginHelper * _helper;

    public:

        simsens::World world;
        simsens::Robot robot;

        bool _collided;

        AutopilotHelper(const char * worldname)
        {
            const auto pwd = getenv(PATH_VARIABLE_NAME);

            char world_path[256] = {};
            sprintf(world_path, "%s/../../worlds/%s.wbt", pwd, worldname);

            char robot_path[256] = {};
            sprintf(robot_path, "%s/../../protos/DiyQuad.proto", pwd);

            simsens::WorldParser::parse(world_path, world, robot_path);

            simsens::RobotParser::parse(robot_path, robot);

            char log_path[256] = {};
            sprintf(log_path, "%s/%s.csv", pwd, worldname);
            _logfile = fopen(log_path, "w");
            fprintf(_logfile, "%s\n", worldname);

            _collided = false;
            
            const auto p = world.robotPose;
            _helper = new PluginHelper({p.x, p.y, p.z, p.phi, p.theta, p.psi});
        }

        ~AutopilotHelper()
        {
            delete _helper;
        }

        hf:: Dynamics::State get_state_from_message(
                const PluginHelper::message_t & message)
        {
            return _helper->get_state_from_message(message);
        }

        simsens::Pose get_pose(const PluginHelper::message_t & message)
        {
            // Use setpoint to get new state
            const auto state = _helper->get_state_from_message(message);

            // Extract pose from state
            const simsens::Pose pose = {
                state.x, state.y, state.z, state.phi, state.theta, state.psi
            };

            // Stop if we detected a collision
            if (world.collided({pose.x, pose.y, pose.z})) {
                dBodySetGravityMode(_helper->robotBody, 1);
                _collided = true;
            }

            // Otherwise, set normally
            _helper->set_dbody_from_state(state);

            return pose;
        }

        bool get_message(PluginHelper::message_t & message)
        {
            return _collided ? false : _helper->get_message(message);
        }

        void write_to_log(
                const simsens::Pose & pose,
                const int * rangefinder_distances,
                const int n_distances)
        {       
            fprintf(_logfile,
                    "%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f",
                    pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi);

            for (int k=0; k<n_distances; ++k) {
                fprintf(_logfile, ",%d", rangefinder_distances[k]);
            }

            fprintf(_logfile, "\n");
        }
};
