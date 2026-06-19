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
#include <sim/vehicles/apexquad.hpp>

class PluginHelper {

    private:

        static constexpr char kRobotName[] = "robot";

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

        auto RunSimulator(const hf::Mode mode, const hf::Setpoint & setpoint)
            -> hf::SimState
        {
            simulator_ = hf::Simulator::Step(simulator_, mode, setpoint,
                    hf::ApexQuad::run);

            return simulator_.dynamics.state;
        }

        auto GetSetpoint() -> hf::Setpoint
        {
            return simulator_.pid_controller.setpoint;
        }

        void SetDbodyFromState(const hf::SimState & state)
        {
            // Negate Y to make leftward positive
            dBodySetPosition(robot_body, state.x, -state.y, state.z);

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
};
