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

static const uint8_t RANGEFINDER_DISPLAY_SCALEUP = 32;

static const char * LOGFILE_NAME =
"/home/levys/Desktop/hackflight/webots/controllers/controller/simsens.csv";

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

    ///////////////////////////////////////////////////////////////////////
    static simsens::SimRangefinder * _simRangefinder;
    static simsens::RangefinderVisualizer * _rangefinderVisualizer;
    static simsens::RobotParser _robotParser;
    static simsens::WorldParser _worldParser;
    static FILE * _logfp;

    // Load world and robot info first time around
    if (!_simRangefinder) {

        char path[1000];

        sprintf(path, "%s/../../worlds/%s.wbt", siminfo.path, siminfo.worldname);
        _worldParser.parse(path);

        sprintf(path, "%s/../../protos/DiyQuad.proto", siminfo.path);
        _robotParser.parse(path);

        _simRangefinder = _robotParser.rangefinders[0];

        _rangefinderVisualizer = new simsens::RangefinderVisualizer(_simRangefinder);

        _logfp = fopen(LOGFILE_NAME, "w");
    }

    // Get simulated rangefinder distances
    int rangefinder_distances_mm[1000] = {}; // arbitrary max size
    int rangefinder_width=0, rangefinder_height=0;
    _simRangefinder->read(
            simsens::pose_t{robot_x, robot_y, robot_z,
            pose.phi, pose.theta, pose.psi},
            _worldParser.walls,
            rangefinder_distances_mm,
            rangefinder_width,
            rangefinder_height);

    for (int k=0; k<rangefinder_width; ++k) {
        fprintf(_logfp, "%d%c", rangefinder_distances_mm[k],
                (k==rangefinder_width-1)?'\n':',');
    }
    fflush(_logfp);


    _rangefinderVisualizer->show(rangefinder_distances_mm, RANGEFINDER_DISPLAY_SCALEUP);

    // Stop if we detected a collision
    const bool debug = true;
    if (simsens::CollisionDetector::detect(
                simsens::vec3_t{robot_x, robot_y, robot_x},
                _worldParser.walls, debug)) {
        return false;
    }

    ///////////////////////////////////////////////////////////////////////

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
