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
#include <simulator/info.h>
#include <simulator/simulator.hpp>

static constexpr char ROBOT_NAME[] = "diyquad";

static dBodyID _robot;

// Platform-independent simulator simulator loop
static Simulator _simulator;

static PidControl _pidControl;

DLLEXPORT void webots_physics_init() 
{
    _robot = dWebotsGetBodyFromDEF(ROBOT_NAME);

    if (_robot == NULL) {

        dWebotsConsolePrintf("webots_physics_init :: ");
        dWebotsConsolePrintf("error : could not get body of robot.\r\n");
    }
    else {

        dBodySetGravityMode(_robot, 0);
    }

    _simulator.init(&_pidControl);
}

DLLEXPORT int webots_physics_collide(dGeomID g1, dGeomID g2) 
{
    (void)g1;
    (void)g2;

    return 0;
}

class PhysicsPluginHelper {

    public:

        static bool get_siminfo(siminfo_t & siminfo)
        {
            if (_robot == NULL) {
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

        static Dynamics::pose_t get_pose_from_siminfo(const siminfo_t & siminfo)
        {
            // Set pose first time around
            static bool _ready;
            if (!_ready) {
                _simulator.setPoseAndFramerate(
                        siminfo.startingPose, siminfo.framerate);
            }
            _ready = true;

            return _simulator.step(siminfo.mode, siminfo.setpoint);
        }

        static void set_dbody_from_pose(const Dynamics::pose_t & pose)
        {
            // Negate Y for leftward positive
            dBodySetPosition(_robot, pose.x, -pose.y, pose.z);

            // Turn Euler angles into quaternion, negating psi for nose-left
            // positive
            const axis3_t euler = { (float)pose.phi, (float)pose.theta, (float)-pose.psi};
            axis4_t quat = euler2quat(euler);

            const dQuaternion q = {quat.w, quat.x, quat.y, quat.z};
            dBodySetQuaternion(_robot, q);
        }

        static FILE * logfile_open(const siminfo_t & siminfo)
        {
            char path[1000];
            sprintf(path, "%s/%s", siminfo.path, siminfo.logfilename);
            return fopen(path, "w");
        }

        static void logfile_write_pose(FILE * logfp, const Dynamics::pose_t & pose)
        {
            fprintf(logfp, "%f,%f,%f,%f,%f,%f", 
                    pose.x,
                    -pose.y, // leftward positive
                    pose.z,
                    pose.phi,
                    pose.theta,
                    -pose.psi); // nose-right positive
        }

    private:

        FILE * logfile;

        static axis4_t euler2quat(const axis3_t & angles)
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
