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

#include "../common.hpp"

#include <simsensors/src/collision.hpp>
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>
#include <simsensors/src/visualizers/rangefinder.hpp>

//static const uint8_t RANGEFINDER_DISPLAY_SCALEUP = 32;

//static FILE * _logfp;

static bool run_normal()
{
    int size = 0;

    siminfo_t siminfo = {};

    // Get sim info from main program
    const auto buffer = (siminfo_t *)dWebotsReceive(&size);

    if (size == sizeof(siminfo_t)) {
        memcpy(&siminfo, buffer, sizeof(siminfo));
    }

    // This happens at startup
    if (siminfo.framerate == 0) {
        return true;
    }

    // Update to get the current pose
    const SimInnerLoop::pose_t pose = _innerLoop.step(siminfo);

    // Turn Euler angles into quaternion, negating psi for nose-right positive 
    const axis3_t euler = { pose.phi, pose.theta, -pose.psi};
    axis4_t quat = {};
    Num::euler2quat(euler, quat);

    const dQuaternion q = {quat.w, quat.x, quat.y, quat.z};
    dBodySetQuaternion(_robot, q);

    // Set robot posed based on state and starting position, negating for
    // rightward negative
    const double robot_x = siminfo.start_x + pose.x;
    const double robot_y = siminfo.start_y - pose.y;
    const double robot_z = siminfo.start_z + pose.z;

    dBodySetPosition(_robot, robot_x, robot_y, robot_z);

    return true;
}

// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    if (_robot == NULL) {
        return;
    }

    static bool _collided;

    if (_collided) {
        dBodySetGravityMode(_robot, 1);
    }
    else {
        _collided = !run_normal();
    }

}
