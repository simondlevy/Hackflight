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

// Webots
#include <plugins/physics.h>

// Hackflight
#define _MAIN
#include <datatypes.h>
#include <simulator/pose.h>
#include <simulator/simulator.hpp>

static dBodyID _robotBody;

class PhysicsPluginHelper {

    private:

        static constexpr char ROBOT_NAME[] = "diyquad";

        // Platform-independent simulator simulator loop
        hf::Simulator _simulator;

    public:

        typedef struct {

            hf::pose_t startingPose;
            float framerate;
            hf::mode_e mode;
            hf::demands_t setpoint;

        } siminfo_t;

        bool get_siminfo(siminfo_t & siminfo)
        {
            if (_robotBody == NULL) {
                return false;
            }

            int bytes_received = 0;

            // Get sim info from main program
            const auto buffer = (siminfo_t *)dWebotsReceive(&bytes_received);

            if (bytes_received == sizeof(siminfo_t)) {
                memcpy(&siminfo, buffer, sizeof(siminfo));
            }

            // Framerate can be zero at startup
            return siminfo.framerate > 0;
        }

        hf::pose_t get_pose_from_siminfo(const siminfo_t & siminfo)
        {
            // Set pose first time around
            static bool _ready;
            if (!_ready) {
                _simulator.init(siminfo.startingPose, siminfo.framerate);
            }
            _ready = true;

            return _simulator.step(siminfo.mode, siminfo.setpoint);
        }

        void set_dbody_from_pose(const hf::pose_t & pose)
        {
            // Negate Y to make leftward positive
            dBodySetPosition(_robotBody, pose.x, -pose.y, pose.z);

            // Turn Euler angles into quaternion, negating psi for nose-left
            // positive
            const hf::axis3_t euler = { (float)pose.phi, (float)pose.theta, (float)-pose.psi};
            const hf::axis4_t quat = euler2quat(euler);

            const dQuaternion q = {quat.w, quat.x, quat.y, quat.z};
            dBodySetQuaternion(_robotBody, q);
        }

        void init()
        {
            _robotBody = dWebotsGetBodyFromDEF(ROBOT_NAME);

            if (_robotBody == NULL) {

                dWebotsConsolePrintf("webots_physics_init :: ");
                dWebotsConsolePrintf("error : could not get body of robot.\r\n");
            }
            else {

                dBodySetGravityMode(_robotBody, 0);
            }
        }

    private:

        static hf::axis4_t euler2quat(const hf::axis3_t & angles)
        {
            // Abbreviations for the various angular functions

            const auto cr = (float)cos(angles.x / 2);
            const auto sr = (float)sin(angles.x / 2);
            const auto cp = (float)cos(angles.y / 2);
            const auto sp = (float)sin(angles.y / 2);
            const auto cy = (float)cos(angles.z / 2);
            const auto sy = (float)sin(angles.z / 2);

            return {
                cr * cp * cy + sr * sp * sy,
                   sr * cp * cy - cr * sp * sy,
                   cr * sp * cy + sr * cp * sy,
                   cr * cp * sy - sr * sp * cy
            };
        }
};
