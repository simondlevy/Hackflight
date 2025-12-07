/* 
 * Custom physics plugin support for Hackflight Webots-based simulator
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
#include <simulator/simulator.hpp>

static constexpr char ROBOT_NAME[] = "diyquad";

static dBodyID _robotBody;

// Platform-independent simulator
static Simulator _simulator;

static ClosedLoopControl _closedLoopControl;

DLLEXPORT void webots_physics_init() 
{
    _robotBody = dWebotsGetBodyFromDEF(ROBOT_NAME);

    if (_robotBody == NULL) {

        dWebotsConsolePrintf("webots_physics_init :: ");
        dWebotsConsolePrintf("error : could not get body of robot.\r\n");
    }
    else {

        dBodySetGravityMode(_robotBody, 0);
    }

    _simulator.init(&_closedLoopControl);
}

// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    if (_robotBody == NULL) {
        return;
    }

    int size = 0;

    Simulator::siminfo_t siminfo = {};

    // Get sim info from main program
    const auto buffer = (Simulator::siminfo_t *)dWebotsReceive(&size);

    if (size == sizeof(Simulator::siminfo_t)) {
        memcpy(&siminfo, buffer, sizeof(siminfo));
    }

    // This happens at startup
    if (siminfo.framerate == 0) {
        return;
    }

    // Update to get the current pose
    const Dynamics::pose_t pose = _simulator.step(siminfo);

    // Turn Euler angles into quaternion, negating psi for nose-right positive 
    const axis3_t euler = { pose.phi, pose.theta, -pose.psi};
    axis4_t quat = {};
    Num::euler2quat(euler, quat);

    const dQuaternion q = {quat.w, quat.x, quat.y, quat.z};
    dBodySetQuaternion(_robotBody, q);

    // Set robot posed based on state and starting position, negating for
    // rightward negative
    dBodySetPosition(
            _robotBody,
            siminfo.start_x + pose.x,
            siminfo.start_y - pose.y,
            siminfo.start_z + pose.z);
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
