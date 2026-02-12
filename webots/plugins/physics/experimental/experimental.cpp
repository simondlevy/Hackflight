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

// C
#include <stdlib.h>
#include <unistd.h>

// Webots
#include "../common.hpp"

// Hackflight
#include <autopilot/rangefinder.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>
#include <simsensors/src/visualizers/rangefinder.hpp>
#include <simsensors/src/world.hpp>

static const uint8_t RANGEFINDER_DISPLAY_SCALEUP = 64;

static const char * LOG_FILE_NAME = "log.csv";

static const char * PATH_VARIABLE_NAME = "WEBOTS_PATH";
static const char * WORLD_VARIABLE_NAME = "WEBOTS_WORLD";

static simsens::Rangefinder * _rangefinder;

static simsens::RangefinderVisualizer * _rangefinderVisualizer;

static char * worldname()
{
    return getenv(WORLD_VARIABLE_NAME);
}

static void load(
        simsens::World & world, simsens::Robot & robot, FILE ** logfpp)
{
    const auto pwd = getenv(PATH_VARIABLE_NAME);

    char path[1000] = {};

    sprintf(path, "%s/../../worlds/%s.wbt", pwd, worldname());
    simsens::WorldParser::parse(path, world);

    sprintf(path, "%s/../../protos/DiyQuad.proto", pwd);
    simsens::RobotParser::parse(path, robot);

    _rangefinder = new simsens::Rangefinder(*robot.rangefinders["VL53L5-forward"]);

    _rangefinderVisualizer = new simsens::RangefinderVisualizer(_rangefinder);

    sprintf(path, "%s/%s", pwd, LOG_FILE_NAME);
    *logfpp = fopen(path, "w");
}

// Returns false on collision, true otherwise
static bool run_normal(PhysicsPluginHelper::siminfo_t & siminfo)
{
    static simsens::World _world;
    static simsens::Robot _robot;
    static FILE * _logfp;
    static int _rangefinder_distances_mm[1000]; // arbitrary max size

    const bool autonomous = siminfo.mode == hf::MODE_AUTONOMOUS;

    // In autonomous mode, use current pose to get setpoints
    if (autonomous) {

        static int frame;

        if (string(worldname()) == "twoexit") {
            hf::RangefinderSetpoint::runTwoExit(frame++,
                    _rangefinder_distances_mm, siminfo.setpoint);
        }

        else {
            printf("No autopilot for world %s\n", worldname());
        }
    }

    // Use setpoints to get new pose
    const auto pose = PhysicsPluginHelper::get_pose_from_siminfo(siminfo);

    // Load world and robot info first time around
    if (!_rangefinder) {
        load(_world, _robot, &_logfp);
    }

    // Get simulated rangefinder distances
    _rangefinder->read(
            simsens::pose_t {
            pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi},
            _world, _rangefinder_distances_mm);

    // Dump everything to logfile
    fprintf(_logfp, "%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f", 
            pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi);
    for (int k=0; k<_rangefinder->getWidth(); ++k) {
        fprintf(_logfp, ",%d", _rangefinder_distances_mm[k]);
    }
    fprintf(_logfp, "\n");

    // Visualize rangefinder distances
    _rangefinderVisualizer->show(_rangefinder_distances_mm,
            RANGEFINDER_DISPLAY_SCALEUP);

    // Stop if we detected a collision
    if (_world.collided({pose.x, pose.y, pose.z})) {
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

        PhysicsPluginHelper::siminfo_t siminfo = {};

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
