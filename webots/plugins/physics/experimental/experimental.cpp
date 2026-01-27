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
#include "../common.hpp"

// Hackflight
#include <autopilot/rangefinder.hpp>

// SimSensors
#include <simsensors/src/collision.hpp>
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>
#include <simsensors/src/visualizers/rangefinder.hpp>
#include <simsensors/src/world.hpp>

static const uint8_t RANGEFINDER_DISPLAY_SCALEUP = 64;

static simsens::Rangefinder * _rangefinder;

static simsens::RangefinderVisualizer * _rangefinderVisualizer;

static void load(const siminfo_t & siminfo,
        simsens::World & world,
        simsens::Robot & robot,
        FILE ** logfpp)
{
    char path[1000];

    sprintf(path, "%s/../../worlds/%s.wbt", siminfo.path, siminfo.worldname);
    simsens::WorldParser::parse(path, world);

    sprintf(path, "%s/../../protos/DiyQuad.proto", siminfo.path);
    simsens::RobotParser::parse(path, robot);

    _rangefinder = new simsens::Rangefinder(*robot.rangefinders[0]);

    _rangefinderVisualizer = new simsens::RangefinderVisualizer(_rangefinder);

    sprintf(path, "%s/%s", siminfo.path, siminfo.poselogname);
    *logfpp = fopen(path, "w");
}

static bool collided(const pose_t & pose, const simsens::World & world)
{
    const bool debug = true;

    printf("y=%+3.3f\n", pose.y);


    return simsens::CollisionDetector::detect(

            // Negate Y for leftward positive
            simsens::vec3_t{pose.x, -pose.y, pose.x},
            world.walls, debug);
}

static void read_rangefinder(
        simsens::Rangefinder & rangefinder,
        simsens::World & world,
        const pose_t & pose,
        int * distances_mm,
        FILE * logfp)
{
    rangefinder.read(
            simsens::pose_t {

            // Negate for leftward positive
            pose.x, -pose.y, pose.z, pose.phi, pose.theta, pose.psi},
            world.walls, distances_mm);

    fprintf(logfp, "%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f", 
            pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi);

    for (int k=0; k<rangefinder.getWidth(); ++k) {
        fprintf(logfp, ",%d", distances_mm[k]);
    }
    fprintf(logfp, "\n");


    fflush(logfp);
}

// Returns false on collision, true otherwise
static bool run_normal(siminfo_t & siminfo)
{
    static simsens::World _world;
    static simsens::Robot _robot;
    static FILE * _logfp;
    static int _rangefinder_distances_mm[1000]; // arbitrary max size

    const bool autonomous = siminfo.mode == MODE_AUTONOMOUS;

    // In autonomous mode, use current pose to get setpoints
    if (autonomous) {
        RangefinderSetpoint::run(_rangefinder_distances_mm, siminfo.setpoint);
    }

    // Use setpoints to get new pose
    const auto pose = PhysicsPluginHelper::get_pose_from_siminfo(siminfo);

    // Load world and robot info first time around
    if (!_rangefinder) {
        load(siminfo, _world, _robot, &_logfp);
    }

    printf("y=%+3.3f\n", pose.y);

    // Get simulated rangefinder distances
    read_rangefinder(*_rangefinder, _world, pose,
            _rangefinder_distances_mm, _logfp);

    _rangefinderVisualizer->show(_rangefinder_distances_mm,
            RANGEFINDER_DISPLAY_SCALEUP, autonomous);

    // Stop if we detected a collision
    if (collided(pose, _world)) {
        return false;
    }

    // Otherwise, set normally
    PhysicsPluginHelper::set_dbody_from_pose(pose);

    return true;
}

// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    static bool _collided;

    if (_collided) {
        dBodySetGravityMode(_robot, 1);
    }

    else {

        siminfo_t siminfo = {};

        if (PhysicsPluginHelper::get_siminfo(siminfo)) {

            if (!run_normal(siminfo)) {
                _collided = true;
            }
        }
    }
}

DLLEXPORT void webots_physics_cleanup() 
{
    delete _rangefinder;
    delete _rangefinderVisualizer;
}
