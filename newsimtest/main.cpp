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

static const uint32_t MAXTIME = 5;
static const float FRAMERATE = 32;

static FILE * openlog(const char * filename, const char * mode)
{
    FILE * fp = fopen(filename, mode);
    if (!fp) {
        fprintf(stderr, "Unable to open file %s for %s\n", filename,
                *mode == 'w' ? "reading" : "writing");
        exit(1);
    }
    return fp;
}

int main(int argc, char ** argv)
{
    if (argc < 3) {
        fprintf(stderr, "Usage: %s WORLDFILE ROBOTFILE\n", argv[0]);
        return 1;
    }

    simsens::WorldParser worldParser = {};
    worldParser.parse(argv[1], "DiyQuad {");

    simsens::RobotParser robotParser = {};
    robotParser.parse(argv[2]);

    const auto pose = worldParser.robotPose;

    Simulator simulator = {};

    simulator.init(
            {pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi}, 
            FRAMERATE);

    auto outputfp = openlog("poselog.csv", "w");

    for (uint32_t t=0; t<MAXTIME*FRAMERATE; ++t) {

        mode_e mode = MODE_HOVERING;

        demands_t setpoint = {};

        const auto pose = simulator.step(mode, setpoint);

        fprintf(outputfp, "%f,%f,%f,%f,%f,%f\n",
                pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi);
    }

    fclose(outputfp);

    return 0;
}
