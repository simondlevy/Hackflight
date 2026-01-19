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

#include <stdio.h>

// Hackflight
#include <datatypes.h>
#include <simulator/simulator.hpp>
#include <vehicles/diyquad.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

static const uint32_t STEPS = 1000;
static const float FRAMERATE = 32;

int main(int argc, char ** argv)
{
    if (argc < 4) {
        fprintf(stderr, "Usage: %s WORLDFILE ROBOTFILE SETPOINTFILE\n",
                argv[0]);
        return 1;
    }

    simsens::WorldParser worldParser = {};
    worldParser.parse(argv[1], "DiyQuad {");

    simsens::RobotParser robotParser = {};
    robotParser.parse(argv[2]);

    FILE * setpointlogfp = fopen(argv[3], "r");

    const auto pose = worldParser.robotPose;

    Simulator simulator = {};

    simulator.init(
            {pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi}, 
            FRAMERATE);

    const auto poselogname = argv[3];
    FILE * poselogfp = fopen(poselogname, "r");
    if (!poselogfp) {
        fprintf(stderr, "Unable to open %s for input\n", poselogname);
        exit(1);
    }

    while (true) {

        char line[1000] = {};

        if (!fgets(line, sizeof(line), setpointlogfp)) {
            break;
        }

        printf("%s", line);

        /*
        if (sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d", 
                &pose.x, &pose.y, &pose.z,
                &pose.phi, &pose.theta, &pose.psi,
                &d[0], &d[1], &d[2], &d[3], &d[4], &d[5], &d[6], &d[7]) == 14) {

        }*/
    }
 
    /*
    for (uint32_t k=0; k<STEPS; ++k) {

        // const auto pose = simulator.step(MODE_IDLE, setpoint);

        fprintf(poselogfp, "%f,%f,%f,%f,%f,%f\n",
                pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi);
    }*/

    fclose(poselogfp);
    fclose(setpointlogfp);

    return 0;
}
