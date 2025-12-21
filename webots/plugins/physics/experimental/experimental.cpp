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
#include <simulator/simulator.hpp>

// Simsensors
#include <sim_datatypes.h>
#include <parsers/world_parser.hpp>
#include <parsers/robot_parser.hpp>
#include <sensors/rangefinder.hpp>
//#include <sensors/rangefinder_visualizer.hpp>

static const uint8_t RANGEFINDER_DISPLAY_SCALEUP = 32;

static constexpr char ROBOT_NAME[] = "diyquad";
static constexpr char BALL_NAME[] = "ball";

static dBodyID _robot;

static dBodyID _ball;

// Platform-independent simulator
static Simulator _simulator;

static PidControl _pidControl;

DLLEXPORT void webots_physics_init() 
{
    _robot = dWebotsGetBodyFromDEF(ROBOT_NAME);

    _ball = dWebotsGetBodyFromDEF(BALL_NAME);

    if (_robot == NULL) {

        dWebotsConsolePrintf("webots_physics_init :: ");
        dWebotsConsolePrintf("error : could not get body of robot.\r\n");
    }
    else {

        dBodySetGravityMode(_robot, 0);
        dBodySetGravityMode(_ball, 0);
    }

    _simulator.init(&_pidControl);
}

// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    if (_robot == NULL) {
        return;
    }

    int size = 0;

    Simulator::info_t siminfo = {};

    // Get sim info from main program
    const auto buffer = (Simulator::info_t *)dWebotsReceive(&size);

    if (size == sizeof(Simulator::info_t)) {
        memcpy(&siminfo, buffer, sizeof(siminfo));
    }

    // This happens at startup
    if (siminfo.framerate == 0) {
        return;
    }

    static simsens::SimRangefinder * _simRangefinder;
    //static simsens::RangefinderVisualizer * _rangefinderVisualizer;
    static simsens::RobotParser _robotParser;
    static simsens::WorldParser _worldParser;

    // Load world and robot info first time around
    if (!_simRangefinder) {

        char path[1000];

        sprintf(path, "%s/../../worlds/%s.wbt", siminfo.path, siminfo.worldname);
        _worldParser.parse(path);

        sprintf(path, "%s/../../protos/DiyQuad.proto", siminfo.path);
        _robotParser.parse(path);

        _simRangefinder = _robotParser.rangefinders[0];

        //_rangefinderVisualizer = new simsens::RangefinderVisualizer(_simRangefinder);
    }

    // Update to get the current pose
    const Simulator::pose_t pose = _simulator.step(siminfo);

    // Set robot posed based on state and starting position, negating for
    // rightward negative
    const double robot_x = siminfo.start_x + pose.x;
    const double robot_y = siminfo.start_y - pose.y;
    const double robot_z = siminfo.start_z + pose.z;

    // Get simulated rangefinder distances
    int ranger_distances_mm[1000] = {}; // arbitrary max size
    simsens::vec3_t endpoint = {};
    _simRangefinder->read(
            simsens::pose_t{robot_x, robot_y, robot_z,
            pose.phi, pose.theta, pose.psi},
            _worldParser.walls, ranger_distances_mm, endpoint);
    // printf("%d\n", ranger_distances_mm[0]);
    //_rangefinderVisualizer->show(ranger_distances_mm, RANGEFINDER_DISPLAY_SCALEUP);

    // Turn Euler angles into quaternion, negating psi for nose-right positive 
    const axis3_t euler = {pose.phi, pose.theta, -pose.psi};
    axis4_t quat = {};
    Num::euler2quat(euler, quat);

    const dQuaternion q = {quat.w, quat.x, quat.y, quat.z};
    dBodySetQuaternion(_robot, q);

    dBodySetPosition(_robot, robot_x, robot_y, robot_z);

    dBodySetPosition(_ball, endpoint.x, endpoint.y, endpoint.z);
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
