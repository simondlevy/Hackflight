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

#include <sim/vehicles/quadx/apexquad.hpp>

#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/world.hpp>

#include "helper.hpp"

class AutopilotHelper {

    private:

        static constexpr char kPathVariableName[] = "WEBOTS_PATH";

        FILE * logfile_;

        PluginHelper * helper_;

    public:

        simsens::World world;
        simsens::Robot robot;

        bool _collided;

        AutopilotHelper(const char * worldname)
        {
            const auto pwd = getenv(kPathVariableName);

            char world_path[256] = {};
            sprintf(world_path, "%s/../../worlds/%s.wbt", pwd, worldname);

            char robot_path[256] = {};
            sprintf(robot_path, "%s/../../protos/DiyQuad.proto", pwd);

            simsens::WorldParser::parse(world_path, world, robot_path);

            simsens::RobotParser::parse(robot_path, robot);

            char log_path[256] = {};
            sprintf(log_path, "%s/%s.csv", pwd, worldname);
            fprintf(logfile_, "%s\n", worldname);

            _collided = false;
            
            const auto p = world.robotPose;
            helper_ = new PluginHelper({p.x, p.y, p.z, p.phi, p.theta, p.psi});
        }

        ~AutopilotHelper()
        {
            delete helper_;
        }

        auto GetState(const PluginHelper::message_t & message) -> hf::SimState
        {
            const auto vparams = hf::ApexQuad::kVehicleParams;

            return helper_->RunSimulator( hf::ApexQuad::Run, vparams,
                    message.mode, message.setpoint);
        }

        auto GetPose(const hf::Mode mode,
                const hf::Setpoint & setpoint) -> simsens::Pose
        {
            const auto vparams = hf::ApexQuad::kVehicleParams;

            const auto state = helper_->RunSimulator( hf::ApexQuad::Run, vparams,
                    mode, setpoint);

            // Extract pose from state
            const simsens::Pose pose = {
                state.x, state.y, state.z, state.phi, state.theta, state.psi
            };

            // Stop if we detected a collision
            if (world.collided({pose.x, pose.y, pose.z})) {
                dBodySetGravityMode(helper_->robot_body, 1);
                _collided = true;
            }

            // Otherwise, set normally
            helper_->SetDbodyFromState(vparams, state);

            return pose;
        }

        auto WriteToLog(
                const simsens::Pose & pose,
                const int * rangefinder_distances,
                const int n_distances)
        {       
            fprintf(logfile_,
                    "%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f",
                    pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi);

            for (int k=0; k<n_distances; ++k) {
                fprintf(logfile_, ",%d", rangefinder_distances[k]);
            }

            fprintf(logfile_, "\n");
        }
};
