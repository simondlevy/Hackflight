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
        const pose_t & pose,
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
        const pose_t & pose,
        int * distances_mm,
        FILE * logfp)
{
    rangefinder.read(
            simsens::pose_t {
            pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi},
            world.walls, distances_mm);

    const auto width = rangefinder.getWidth();

    for (int k=0; k<width; ++k) {
        fprintf(logfp, "%d%c", distances_mm[k], (k==width-1)?'\n':',');
    }

    fflush(logfp);

    visualizer.show(distances_mm, RANGEFINDER_DISPLAY_SCALEUP);
}

static void get_setpoint_from_rangefinder(const int * rangefinder_distances_mm,
        demands_t & setpoint)
{
    const auto d = rangefinder_distances_mm;

    /*
    printf("d0=%d d1=%d d2=%d d3=%d d4=%d d5=%d d6=%d d7=%d\n",
            d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
            */

    const bool center_is_clear = d[3] == -1 && d[4] == -1;

    setpoint.pitch = center_is_clear ? 0.4 : 0;
    setpoint.yaw = center_is_clear ? 0 : 0.2;
}

// Returns false on collision, true otherwise
static bool run_normal(siminfo_t & siminfo)
{
    static simsens::Rangefinder * _rangefinder;
    static simsens::RangefinderVisualizer * _rangefinderVisualizer;
    static simsens::WorldParser _worldParser;
    static FILE * _logfp;
    static int _rangefinder_distances_mm[1000]; // arbitrary max size

    // In autonomous mode, use current pose to get setpoints
    if (siminfo.flightMode == MODE_AUTONOMOUS) {
        get_setpoint_from_rangefinder(_rangefinder_distances_mm,
                siminfo.setpoint);
    }

    // Use setpoints to get new pose
    const auto pose = _innerLoop.step(siminfo);

    // Load world and robot info first time around
    if (!_rangefinder) {
        load(siminfo, _worldParser, &_rangefinder, &_rangefinderVisualizer,
                &_logfp);
    }

    // Get simulated rangefinder distances
    read_rangefinder(*_rangefinder, *_rangefinderVisualizer, _worldParser,
            pose, _rangefinder_distances_mm, _logfp);

    // Stop if we detected a collision
    if (collided(pose, _worldParser)) {
        return false;
    }

    // Otherwise, set normally
    set_dbody_from_pose(pose);

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

            if (!run_normal(siminfo)) {
                _collided = true;
            }
        }
    }
}
