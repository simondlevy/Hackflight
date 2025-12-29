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

static void load(const siminfo_t & siminfo,
        simsens::WorldParser & worldParser,
        simsens::Rangefinder ** rangefinder,
        simsens::RangefinderVisualizer ** rangefinderVisualizer,
        FILE ** logfpp)
{
    char path[1000];

    sprintf(path, "%s/../../worlds/%s.wbt", siminfo.path, siminfo.worldname);
    worldParser.parse(path);

    sprintf(path, "%s/../../protos/DiyQuad.proto", siminfo.path);
    simsens::RobotParser robotParser = {};
    robotParser.parse(path);

    *rangefinder = robotParser.rangefinders[0];

    *rangefinderVisualizer = new simsens::RangefinderVisualizer(*rangefinder);

    *logfpp = fopen(LOGFILE_NAME, "w");
}

static bool collided(
        const SimInnerLoop::pose_t & pose,
        const simsens::WorldParser & worldParser)
{
    const bool debug = true;

    if (simsens::CollisionDetector::detect(
                simsens::vec3_t{pose.x, pose.y, pose.x},
                worldParser.walls, debug)) {
        return true;
    }

    return false;
}

static void read_rangefinder(
        simsens::Rangefinder & rangefinder,
        simsens::RangefinderVisualizer & visualizer,
        simsens::WorldParser & world,
        const SimInnerLoop::pose_t & pose,
        int * distances_mm,
        FILE * logfp)
{
    int width=0, height=0;

    rangefinder.read(
            simsens::pose_t{
            pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi},
            world.walls,
            distances_mm,
            width,
            height);

    for (int k=0; k<width; ++k) {
        fprintf(logfp, "%d%c", distances_mm[k], (k==width-1)?'\n':',');
    }

    fflush(logfp);

    visualizer.show(distances_mm, RANGEFINDER_DISPLAY_SCALEUP);
}

static bool run_normal(
        const siminfo_t & siminfo, const SimInnerLoop::pose_t pose)
{
    static simsens::Rangefinder * _rangefinder;
    static simsens::RangefinderVisualizer * _rangefinderVisualizer;
    static simsens::WorldParser _worldParser;
    static FILE * _logfp;

    // Load world and robot info first time around
    if (!_rangefinder) {
        load(siminfo, _worldParser, &_rangefinder, &_rangefinderVisualizer,
                &_logfp);
    }

    // Get simulated rangefinder distances
    int rangefinder_distances_mm[1000] = {}; // arbitrary max size
    read_rangefinder(*_rangefinder, *_rangefinderVisualizer, _worldParser,
            pose, rangefinder_distances_mm, _logfp);

    // Stop if we detected a collision
    if (collided(pose, _worldParser)) {
        return false;
    }

    dBodySetPosition(_robot, pose.x, pose.y, pose.z);

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

        siminfo_t siminfo = {};

        if (get_siminfo(siminfo)) {

            if (siminfo.flightMode == MODE_AUTONOMOUS) {
                siminfo.setpoint.thrust = 0.5;
                siminfo.setpoint.roll = 0;
                siminfo.setpoint.pitch = 0;
                siminfo.setpoint.yaw = 0;
            }

            SimInnerLoop::pose_t pose = {};

            get_pose(siminfo, pose);

            if (!run_normal(siminfo, pose)) {
                _collided = true;
            }
        }
    }
}
