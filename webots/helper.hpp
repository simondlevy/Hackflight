/* 
 * Webots custom physics plugin support for Hackflight
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

// C 
#include <stdio.h>

// Webots
#include <plugins/physics.h>

// Hackflight
#define _MAIN
#include <sim/dynamics.hpp>
#include <sim/simulator.hpp>

class PluginHelper {

    private:

        static constexpr char kRobotName[] = "robot";

        static constexpr char kPathVariableName[] = "WEBOTS_PATH";

        static constexpr char kWorldVariableName[] = "WORLD_FILE";

        // Platform-independent simulator simulator loop
        hf::Simulator simulator_;

    public:

        typedef struct {

            hf::Mode mode;
            hf::Setpoint setpoint;

        } message_t;

        dBodyID robot_body;

        PluginHelper() : PluginHelper({0, 0, 0, 0, 0, 0}) {}

        PluginHelper(const hf::Pose & starting_pose)
        {
            simulator_ = hf::Simulator(starting_pose);

            robot_body = InitBody(kRobotName);

            const auto pwd = getenv(kPathVariableName);
            char log_path[256] = {};
            sprintf(log_path, "%s/log.csv", pwd);
            logfile_ = fopen(log_path, "w");
            fprintf(logfile_, "%s\n", getenv(kWorldVariableName));
        }

        static dBodyID InitBody(const char * name)
        {
            auto body = dWebotsGetBodyFromDEF(name);

            if (body == NULL) {

                dWebotsConsolePrintf("webots_physics_init :: ");
                dWebotsConsolePrintf("error : could not get body of %s.\r\n", name);
            }
            else {

                dBodySetGravityMode(body, 0);
            }

            return body;
         }

        // Get sim info from main program
        static auto GetMessage() -> message_t
        {
            int bytes_received = 0;

            const auto buffer = (message_t *)dWebotsReceive(&bytes_received);

            message_t message = {};

            const auto ready = bytes_received == sizeof(message_t);

            if (ready) {
                memcpy(&message, buffer, sizeof(message));
            }

            return message;
        }

        auto RunSimulator(
                hf::EffectorFun effector_fun,
                const hf::VehicleParams & vehicle_params,
                const hf::Mode mode,
                const hf::Setpoint & setpoint) -> hf::SimState
        {
            simulator_ = hf::Simulator::Step(simulator_, mode, setpoint,
                    effector_fun, vehicle_params);

            const auto state = simulator_.dynamics.state;

            return state;
        }

        auto GetSetpoint() -> hf::Setpoint
        {
            return simulator_.pid_controller.setpoint;
        }

        void SetDbodyFromState(const hf::VehicleParams vparams, const hf::SimState & state)
        {
            // Add leg height to altitude to avoid sinking below floor
            const auto z = state.z + vparams.leg_height;

            // Negate Y to make leftward positive
            dBodySetPosition(robot_body, state.x, -state.y, z);

            fprintf(logfile_, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                    dWebotsGetTime()/1000, state.x, state.dx, state.y,
                    state.dy, z, state.dz, state.phi, state.dphi, state.theta,
                    state.dtheta, state.psi, state.dpsi);

            fflush(logfile_);

            // Turn Euler angles into quaternion, negating psi for nose-left
            // positive

            const auto cr = (float)cos(state.phi / 2);
            const auto sr = (float)sin(state.phi / 2);
            const auto cp = (float)cos(state.theta / 2);
            const auto sp = (float)sin(state.theta / 2);
            const auto cy = (float)cos(-state.psi / 2);
            const auto sy = (float)sin(-state.psi / 2);

            const dQuaternion q = {
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy
            };

            dBodySetQuaternion(robot_body, q);
        }

    private:

        FILE * logfile_;
};
