/* 
 * Webots experimental custom physics plugin support for Hackflight
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

class ExperimentalHelper {

    private:

        static constexpr char PATH_VARIABLE_NAME[] = "WEBOTS_PATH";
        static constexpr char LOG_FILE_NAME[] = "log.csv";

        FILE * _logfile;

        PluginHelper * _helper;

    public:

        simsens::World world;
        simsens::Robot robot;

        bool _collided;

        ExperimentalHelper(const char * worldname)
        {
            const auto pwd = getenv(PATH_VARIABLE_NAME);

            char path[1000] = {};

            sprintf(path, "%s/../../worlds/%s.wbt", pwd, worldname);
            simsens::WorldParser::parse(path, world);

            sprintf(path, "%s/../../protos/DiyQuad.proto", pwd);
            simsens::RobotParser::parse(path, robot);

            sprintf(path, "%s/%s", pwd, LOG_FILE_NAME);
            _logfile = fopen(path, "w");

            _collided = false;
            
            _helper = new PluginHelper();
        }

        ~ExperimentalHelper()
        {
            delete _helper;
        }

        hf:: Dynamics::state_t get_state_from_siminfo(const PluginHelper::siminfo_t & siminfo)
        {
            return _helper->get_state_from_siminfo(siminfo);
        }

        simsens::pose_t get_pose(const PluginHelper::siminfo_t & siminfo)
        {
            // Use setpoint to get new state
            const auto newstate = _helper->get_state_from_siminfo(siminfo);

            // Extract pose from newstate
            const simsens::pose_t pose = {
                newstate.x, newstate.y, newstate.z, newstate.phi, newstate.theta, newstate.psi
            };

            // Stop if we detected a collision
            if (world.collided({pose.x, pose.y, pose.z})) {
                dBodySetGravityMode(_helper->robotBody, 1);
                _collided = true;
            }

            // Otherwise, set normally
            _helper->set_dbody_from_state(newstate);

            return pose;
        }

        bool get_siminfo(PluginHelper::siminfo_t & siminfo)
        {
            return _helper->get_siminfo(siminfo);
        }

        bool collided()
        {
            return _collided;
        }

        void write_to_log(
                const simsens::pose_t & pose,
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
