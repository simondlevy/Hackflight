/**
 * Copyright (C) 2026 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

// C/C++
#include <stdio.h>

// Hackflight
#include <autopilot/rangefinder.hpp>
#include <datatypes.h>
#include <simulator/simulator.hpp>
#include <vehicles/diyquad.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

static const float MAXTIME = 10;
static const float HOVERTIME = 2;
static const float FRAMERATE = 32;

int main(int argc, char ** argv)
{
    if (argc < 3) {
        fprintf(stderr, "Usage: %s WORLDFILE ROBOTFILE\n", argv[0]);
        return 1;
    }

    simsens::Robot robot = {};
    simsens::RobotParser::parse(argv[1], robot);

    simsens::World world = {};
    simsens::WorldParser::parse(argv[2], world, "DiyQuad {");

    simsens::Rangefinder rangefinder =
        simsens::Rangefinder(*robot.rangefinders[0]);

    const auto pose = world.getRobotPose();

    Simulator simulator = {};

    simulator.init(
            {pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi}, 
            FRAMERATE);

    const char * LOGNAME = "poselog.csv";
    auto  * outfp = fopen(LOGNAME, "w");
    if (!outfp) {
        fprintf(stderr, "Unable to open file %s for wirting\n", LOGNAME);
        exit(1);
    }
 
    int rangefinder_distances_mm[1000] = {}; // arbitrary max size

    for (uint32_t t=0; t<MAXTIME*FRAMERATE; ++t) {

        const auto mode = t < HOVERTIME*FRAMERATE ? MODE_HOVERING :
            MODE_AUTONOMOUS;

        demands_t setpoint = {};

        RangefinderSetpoint::run(rangefinder_distances_mm, setpoint);

        const auto pose = simulator.step(mode, setpoint);

        if (world.collided({pose.x, pose.y, pose.x})) {
            printf("collision!\n");
            break;
        }

        // Get simulated rangefinder distances
        rangefinder.read(
                simsens::pose_t {
                pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi},
                world, rangefinder_distances_mm);

        // Dump everything to logfile
        fprintf(outfp, "%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f", 
                pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi);
        for (int k=0; k<rangefinder.getWidth(); ++k) {
            fprintf(outfp, ",%d", rangefinder_distances_mm[k]);
        }
        fprintf(outfp, "\n");

    }

    fclose(outfp);

    return 0;
}
