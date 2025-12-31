/* 
 * Custom physics plugin custom for Hackflight Webots-based simulator
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
#include <pid.hpp>
#include <simulator/inner.hpp>

static constexpr char ROBOT_NAME[] = "diyquad";

static dBodyID _robot;

// Platform-independent simulator inner loop
static SimInnerLoop _innerLoop;

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

    _innerLoop.init(&_pidControl);
}


DLLEXPORT int webots_physics_collide(dGeomID g1, dGeomID g2) 
{
    (void)g1;
    (void)g2;

    return 0;
}

DLLEXPORT void webots_physics_cleanup() 
{
}

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

static void get_pose(const siminfo_t & siminfo, pose_t & pose)
{
    // Update to get the current pose
    _innerLoop.step(siminfo, pose);

    //pose.psi -= 2.44343;

    // Turn Euler angles into quaternion, negating psi for nose-right positive 
    const axis3_t euler = { pose.phi, pose.theta, -pose.psi};
    axis4_t quat = {};
    Num::euler2quat(euler, quat);

    const dQuaternion q = {quat.w, quat.x, quat.y, quat.z};
    dBodySetQuaternion(_robot, q);

    // Set robot posed based on state and starting position, negating for
    // rightward negative
    pose.y = -pose.y;
    pose.x += siminfo.startingPose.x;
    pose.y += siminfo.startingPose.y;
    pose.z += siminfo.startingPose.z;
}
