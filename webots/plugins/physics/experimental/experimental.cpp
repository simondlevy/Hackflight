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
#include <simsensors/src/collision.hpp>
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>
//#include <simsensors/src/visualizers/rangefinder.hpp>

static const uint8_t RANGEFINDER_DISPLAY_SCALEUP = 32;

static constexpr char ROBOT_NAME[] = "diyquad";

static dBodyID _robot;

static dBodyID _red_ball;

// Platform-independent simulator
static Simulator _simulator;

static PidControl _pidControl;

static FILE * _logfp;

static bool run_normal()
{
    // Get sim info from main program
    int bytes_received = 0;
    siminfo_t siminfo = {};
    const auto buffer = (siminfo_t *)dWebotsReceive(&bytes_received);
    if (bytes_received == sizeof(siminfo_t)) {
        memcpy(&siminfo, buffer, sizeof(siminfo));
    }

    // This happens at startup
    if (siminfo.framerate == 0) {
        return true;
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

    if (simsens::CollisionDetector::detect(
                simsens::vec3_t{robot_x, robot_y, robot_x},
                _worldParser.walls)) {
        return false;
    }

    // Get simulated rangefinder distances
    int ranger_distances_mm[1000] = {}; // arbitrary max size
    simsens::vec3_t dbg_intersection = {};
    _simRangefinder->read(
            simsens::pose_t{robot_x, robot_y, robot_z,
            pose.phi, pose.theta, pose.psi},
            _worldParser.walls,
            ranger_distances_mm,
            &dbg_intersection,
            _logfp);

    //_rangefinderVisualizer->show(ranger_distances_mm, RANGEFINDER_DISPLAY_SCALEUP);

    // Turn Euler angles into quaternion, negating psi for nose-right positive 
    const axis3_t euler = {pose.phi, pose.theta, -pose.psi};
    axis4_t quat = {};
    Num::euler2quat(euler, quat);

    const dQuaternion q = {quat.w, quat.x, quat.y, quat.z};
    dBodySetQuaternion(_robot, q);

    dBodySetPosition(_robot, robot_x, robot_y, robot_z);

    if (_red_ball) {
        dBodySetPosition(_red_ball,
                dbg_intersection.x, dbg_intersection.y, dbg_intersection.z);
    }

    return true;
}

DLLEXPORT void webots_physics_init() 
{
    _robot = dWebotsGetBodyFromDEF(ROBOT_NAME);

    _red_ball = dWebotsGetBodyFromDEF("red_ball");

    if (_robot == NULL) {

        dWebotsConsolePrintf("webots_physics_init :: ");
        dWebotsConsolePrintf("error : could not get body of robot.\r\n");
    }
    else {

        dBodySetGravityMode(_robot, 0);
        if (_red_ball) {
            dBodySetGravityMode(_red_ball, 0);
        }
    }

    _simulator.init(&_pidControl);

    _logfp = fopen(
            "/home/levys/Desktop/hackflight/webots/controllers/experimental/simsens.csv", "w");
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
        if (_red_ball) {
            dBodySetPosition(_red_ball, 0, 0, -1);
        }
    }
    else {
        _collided = !run_normal();
    }
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
