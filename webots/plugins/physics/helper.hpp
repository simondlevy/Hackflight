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

        static constexpr char ROBOT_NAME[] = "diyquad";

        // Platform-independent simulator simulator loop
        hf::Simulator _simulator;

    public:

        typedef struct {

            hf::mode_e mode;
            hf::Setpoint setpoint;

        } message_t;

        dBodyID robotBody;
       
        PluginHelper() : PluginHelper({0, 0, 0, 0, 0, 0}) {}

        PluginHelper(const hf::pose_t & startingPose)
        {
            _simulator = hf::Simulator(startingPose);

            robotBody = dWebotsGetBodyFromDEF(ROBOT_NAME);

            if (robotBody == NULL) {

                dWebotsConsolePrintf("webots_physics_init :: ");
                dWebotsConsolePrintf("error : could not get body of robot.\r\n");
            }
            else {

                dBodySetGravityMode(robotBody, 0);
            }
        }

        // Get sim info from main program
        static auto get_message() -> message_t
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

        auto run_simulator(const hf::mode_e mode, const hf::Setpoint & setpoint)
            -> hf::SimState
        {
            _simulator = hf::Simulator::step(_simulator, mode, setpoint);

            return _simulator.dynamics.state;
        }

        auto set_dbody_from_state(const hf::SimState & state)
        {
            // Negate Y to make leftward positive
            dBodySetPosition(robotBody, state.x, -state.y, state.z);

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

            dBodySetQuaternion(robotBody, q);
        }
};
